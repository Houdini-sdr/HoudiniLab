#!/usr/bin/env python3
"""Live CSI web dashboard for the RENEW/Houdini sounder.

Runs the sounder in *viewing mode* (``sounder --view``), which computes per-antenna
CSI in C++ (pilot-agnostic -- uses the config's freq-domain reference, so LTS /
Zadoff-Chu / any ``pilot_seq`` works) and streams one UDP datagram per (frame,
antenna). This backend receives those datagrams, and serves a self-contained web
page (Server-Sent Events + HTML5 canvas -- no external libraries) that shows, per
RX antenna, live magnitude & phase across subcarriers plus a scrolling waterfall.
Scales automatically to however many antennas appear in the stream.

Usage (on the DGX, then browse via SSH port-forward ``-L 8080:localhost:8080``):

    # A) backend launches the sounder itself:
    python3 csi_server.py --launch --conf files/houdini-1u.json

    # B) backend only receives (you run `sounder --view` yourself):
    python3 csi_server.py

Wire format (little-endian): [magic u32 'CSI1'][frame u32][ant u32][num_sc u32]
[rate f32] then num_sc * (H_re f32, H_im f32).
"""
import argparse
import json
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAGIC_CSI = 0x43534931  # "CSI1" -- pilot channel estimate
MAGIC_CNS = 0x434E5331  # "CNS1" -- equalized uplink-data constellation
CSI_HDR = struct.Struct("<IIIIf")  # magic, frame, ant, num_sc, rate
CNS_HDR = struct.Struct("<IIIII")  # magic, frame, ant, num_pts, mod_order

# ---- shared state: latest CSI + constellation per antenna ------------------
_lock = threading.Lock()
_latest = {}   # ant_id -> {"csi": {...}, "cns": {...}}
_seq = 0       # bumps on every new datagram so the SSE loop knows there's fresh data
_stats = {"pkts": 0, "t0": time.time()}


def _parse_csi(payload):
    magic, frame, ant, nsc, rate = CSI_HDR.unpack_from(payload, 0)
    off = CSI_HDR.size
    if len(payload) < off + 8 * nsc:
        return None
    vals = struct.unpack_from("<%df" % (2 * nsc), payload, off)
    mag_db, phase, mags = [], [], []
    for k in range(nsc):
        re, im = vals[2 * k], vals[2 * k + 1]
        m = math.hypot(re, im)
        mags.append(m)
        if m < 1e-9:               # unused subcarrier (guard band / DC null)
            mag_db.append(None)
            phase.append(None)
        else:
            mag_db.append(20.0 * math.log10(m))
            phase.append(math.atan2(im, re))
    peak = max((m for m in mags if m > 0), default=0.0)
    return int(ant), {"frame": int(frame), "sc": int(nsc), "rate": float(rate),
                      "mag_db": mag_db, "phase": phase,
                      "peak_db": (20.0 * math.log10(peak) if peak > 0 else 0.0)}


def _parse_cns(payload):
    magic, frame, ant, npt, mod = CNS_HDR.unpack_from(payload, 0)
    off = CNS_HDR.size
    if len(payload) < off + 8 * npt:
        return None
    vals = struct.unpack_from("<%df" % (2 * npt), payload, off)
    pts = [[vals[2 * i], vals[2 * i + 1]] for i in range(npt)]
    return int(ant), {"frame": int(frame), "mod": int(mod), "pts": pts}


def _udp_loop(bind_host, bind_port):
    global _seq
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((bind_host, bind_port))
    print("[csi] listening on %s:%d" % (bind_host, bind_port), flush=True)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except OSError:
            break
        if len(data) < 4:
            continue
        magic = struct.unpack_from("<I", data, 0)[0]
        parsed, kind = (None, None)
        if magic == MAGIC_CSI:
            parsed, kind = _parse_csi(data), "csi"
        elif magic == MAGIC_CNS:
            parsed, kind = _parse_cns(data), "cns"
        if parsed is None:
            continue
        ant, rec = parsed
        with _lock:
            slot = _latest.setdefault(ant, {})
            slot[kind] = rec
            slot["t"] = time.monotonic()   # last datagram seen for this antenna
            _seq += 1
            _stats["pkts"] += 1


def _snapshot():
    """Latest record per antenna, each stamped with how old it is.

    The sounder drops slots whose samples carry RX gap padding, so a lossy link
    simply stops producing datagrams rather than sending bad ones. Without an age
    the page cannot tell a frozen panel from a healthy static channel, which on a
    stationary bench look identical.
    """
    now = time.monotonic()
    with _lock:
        out = {}
        for a, slot in _latest.items():
            rec = {k: v for k, v in slot.items() if k != "t"}
            rec["age_ms"] = int(max(0.0, now - slot.get("t", now)) * 1000)
            out[str(a)] = rec
        return _seq, out


# ---- HTTP / SSE ------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):  # quiet
        pass

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = (PAGE.replace("__STALE_MS__", str(self.server.stale_ms))
                        .replace("__MAG_TOP__", str(self.server.mag_top))
                        .replace("__MAG_SPAN__", str(self.server.mag_span))
                        .encode("utf-8"))
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/stream"):
            self._sse()
        else:
            self.send_error(404)

    def _sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        # close-delimited stream (no Content-Length): EventSource reads events as
        # they arrive and reconnects if the connection drops.
        self.send_header("Connection", "close")
        self.end_headers()
        fps = self.server.fps
        stale_ms = self.server.stale_ms
        last_seq = -1
        last_stale_push = 0.0
        try:
            while True:
                seq, snap = _snapshot()
                # While the stream is stalled _seq never moves, so pushing only on
                # a seq change would freeze the age on screen too and the badge
                # would never appear. Re-send slowly whenever anything is stale.
                now = time.monotonic()
                stale = any(r.get("age_ms", 0) >= stale_ms for r in snap.values())
                if snap and (seq != last_seq or
                             (stale and now - last_stale_push >= 0.5)):
                    last_seq = seq
                    last_stale_push = now
                    msg = "data: %s\n\n" % json.dumps({"ant": snap})
                    self.wfile.write(msg.encode("utf-8"))
                    self.wfile.flush()
                else:
                    # keep-alive comment so proxies/clients don't time out
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                time.sleep(1.0 / fps)
        except (BrokenPipeError, ConnectionResetError):
            return


# ---- sounder launcher ------------------------------------------------------
def _topology_of(sounder_dir, conf):
    """The topology file a config names, or None if it cannot be determined.

    Falling back to None is fine: teardown_framer.py then uses its own default,
    which is the same file every shipped config points at.
    """
    try:
        path = conf if os.path.isabs(conf) else os.path.join(sounder_dir, conf)
        with open(path, encoding="utf-8") as f:
            return json.load(f).get("serial_file") or None
    except (OSError, ValueError):
        return None


def _launch_sounder(args, udp_dest):
    """Run the sounder in viewing mode on this host (teardown + env + retries)."""
    env = os.environ.copy()
    env["HOUDINI_CSI_UDP"] = udp_dest
    env["HOUDINI_MAX_FRAME"] = str(args.max_frame)
    if args.csi_fps:
        env["HOUDINI_CSI_FPS"] = str(args.csi_fps)
    sd = args.sounder_dir
    # Tear down against the radios THIS run will use: the config names its own
    # topology file, so a config pointed at a different bench tears down that
    # bench rather than whatever the default topology happens to list.
    topo = _topology_of(sd, args.conf)
    td = 'csi_gui/teardown_framer.py' + (' --topology "%s"' % topo if topo else '')
    # A tiny shell wrapper: teardown any stuck framer, then run sounder --view,
    # retrying the flaky cold-start. Mirrors the HIL test harness. The teardown's
    # output is kept (not sent to /dev/null): it exits non-zero when a radio could
    # not be cleared, and that is usually the reason the sounder then fails.
    script = (
        'source "%s"/bin/activate 2>/dev/null; '
        'export LD_LIBRARY_PATH="%s"/lib '
        'SOAPY_SDR_PLUGIN_PATH="%s"/lib/SoapySDR/modules0.8-3; '
        'cd "%s"; '
        'for a in 1 2 3 4; do '
        '  timeout 60 python3 %s 2>&1 | sed -u "s/^/[teardown] /"; sleep 8; '
        '  ./build/sounder --view --conf_file "%s" --storepath "%s" 2>&1 | '
        '     sed -u "s/^/[sounder] /"; '
        '  echo "[sounder] exited, retrying..."; sleep 5; '
        'done'
    ) % (args.venv, args.venv, args.venv, sd, td, args.conf, args.storepath)
    print("[csi] launching sounder --view in %s" % sd, flush=True)
    return subprocess.Popen(["bash", "-lc", script], env=env,
                            preexec_fn=os.setsid)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--udp-host", default="0.0.0.0", help="CSI UDP bind host")
    ap.add_argument("--udp-port", type=int, default=9999, help="CSI UDP bind port")
    ap.add_argument("--http-host", default="0.0.0.0", help="web server bind host")
    ap.add_argument("--http-port", type=int, default=8080, help="web server port")
    ap.add_argument("--fps", type=float, default=30.0, help="dashboard push rate")
    ap.add_argument("--mag-top", type=float, default=90.0,
                    help="top of the FIXED |H| axis in dB (default: %(default)s)")
    ap.add_argument("--mag-span", type=float, default=40.0,
                    help="height of the FIXED |H| axis in dB (default: %(default)s). "
                         "Both panels are fixed frame to frame; widen the span if "
                         "the trace runs off scale")
    ap.add_argument("--stale-ms", type=int, default=1500,
                    help="dim an antenna's plots when its last update is older "
                         "than this. Raise it if you lower --csi-fps, or every "
                         "card will read as stale (default: %(default)s)")
    ap.add_argument("--launch", action="store_true",
                    help="also launch the sounder in viewing mode on this host")
    ap.add_argument("--sounder-dir", default=os.path.expanduser("~/repos/HoudiniLab/CC/Sounder"))
    ap.add_argument("--venv", default=os.path.expanduser("~/houdini_test"),
                    help="virtualenv prefix holding SoapySDR and the Houdini "
                         "plugin, used when --launch runs the sounder")
    ap.add_argument("--conf", default="files/houdini-1u.json")
    ap.add_argument("--storepath", default="/tmp/houdini_hdf5")
    ap.add_argument("--max-frame", type=int, default=2_000_000_000,
                    help="HOUDINI_MAX_FRAME for continuous viewing")
    ap.add_argument("--csi-fps", type=float, default=0.0,
                    help="HOUDINI_CSI_FPS per-antenna stream rate (0 = sounder default 30)")
    ap.add_argument("--dest-host", default="127.0.0.1",
                    help="host the sounder streams CSI to (when --launch)")
    args = ap.parse_args()

    t = threading.Thread(target=_udp_loop, args=(args.udp_host, args.udp_port),
                         daemon=True)
    t.start()

    child = None
    if args.launch:
        child = _launch_sounder(args, "%s:%d" % (args.dest_host, args.udp_port))

    def _stats_loop():
        while True:
            time.sleep(5.0)
            with _lock:
                n = _stats["pkts"]
                ants = sorted(_latest.keys())
            print("[csi] %d datagrams, antennas=%s" % (n, ants), flush=True)
    threading.Thread(target=_stats_loop, daemon=True).start()

    srv = ThreadingHTTPServer((args.http_host, args.http_port), Handler)
    srv.fps = args.fps
    srv.stale_ms = args.stale_ms
    srv.mag_top = args.mag_top
    srv.mag_span = args.mag_span
    srv.daemon_threads = True
    # Serve in a daemon thread so the main thread can wait for Ctrl+C. (Calling
    # srv.shutdown() from a signal handler on the serve_forever thread deadlocks.)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = "http://localhost:%d/" % args.http_port
    print("[csi] dashboard at %s  (SSH: -L %d:localhost:%d)"
          % (url, args.http_port, args.http_port), flush=True)

    def _cleanup():
        if child is not None:  # kill the whole sounder process group
            try:
                os.killpg(os.getpgid(child.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass

    def _sigterm(*_):
        print("\n[csi] shutting down (SIGTERM)", flush=True)
        _cleanup()
        os._exit(0)
    signal.signal(signal.SIGTERM, _sigterm)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[csi] shutting down (Ctrl+C)", flush=True)
        _cleanup()
        os._exit(0)  # hard exit: avoids any serve_forever/shutdown deadlock


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Houdini live CSI</title>
<style>
 body{font-family:system-ui,Arial,sans-serif;margin:0;background:#0e1116;color:#e6edf3}
 header{padding:10px 16px;background:#161b22;border-bottom:1px solid #30363d;
        display:flex;gap:16px;align-items:baseline}
 header h1{font-size:16px;margin:0;font-weight:600}
 header .meta{font-size:12px;color:#8b949e}
 #ants{display:flex;flex-wrap:wrap;gap:14px;padding:14px}
 .card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px;width:520px}
 .card h2{font-size:13px;margin:0 0 8px;color:#58a6ff}
 .card.stale .row{opacity:.4}
 .card h2 .badge{display:none;margin-left:8px;font-size:11px;font-weight:500;
        padding:2px 6px;border-radius:10px;background:#3d2d16;color:#e3b341;
        border:1px solid #6b4f1d;vertical-align:middle}
 .card.stale h2 .badge{display:inline-block}
 .row{display:flex;gap:8px}
 .plot{position:relative}
 .plot .lbl{position:absolute;top:2px;left:6px;font-size:10px;color:#8b949e}
 canvas{background:#0b0f14;border:1px solid #21262d;border-radius:4px;display:block}
 .status{font-size:11px;color:#8b949e;margin-top:6px}
 .off{color:#f85149}
</style></head>
<body>
<header>
  <h1>Houdini live CSI</h1>
  <span class="meta" id="meta">connecting…</span>
</header>
<div id="ants"></div>
<script>
const W=250,H=120,WFW=250,WFH=120;   // plot sizes
const STALE_MS=__STALE_MS__;         // no update for this long -> dim + badge
// Both top panels are FIXED frame to frame. An axis that re-ranges per frame makes
// a static channel look alive and hides real drift, so nothing here auto-scales.
const MAG_TOP=__MAG_TOP__, MAG_BOT=__MAG_TOP__-__MAG_SPAN__;
const cards={};                       // ant_id -> {mag,phase,wf,wfimg,wfrow,frame}

function jet(v){ // v in [0,1] -> [r,g,b]
  v=Math.max(0,Math.min(1,v));
  const r=Math.max(0,Math.min(1,1.5-Math.abs(4*v-3)));
  const g=Math.max(0,Math.min(1,1.5-Math.abs(4*v-2)));
  const b=Math.max(0,Math.min(1,1.5-Math.abs(4*v-1)));
  return [r*255|0,g*255|0,b*255|0];
}

function makeCard(ant){
  const wrap=document.createElement('div'); wrap.className='card';
  wrap.innerHTML=`<h2>RX antenna ${ant}<span class="badge"></span></h2>
    <div class="row">
      <div class="plot"><span class="lbl">|H| (dB) vs subcarrier</span>
        <canvas width="${W}" height="${H}"></canvas></div>
      <div class="plot"><span class="lbl">phase (rad)</span>
        <canvas width="${W}" height="${H}"></canvas></div>
    </div>
    <div class="row" style="margin-top:8px">
      <div class="plot"><span class="lbl">waterfall |H| (time ↓)</span>
        <canvas width="${WFW}" height="${WFH}"></canvas></div>
      <div class="plot"><span class="lbl">constellation (equalized U)</span>
        <canvas width="${WFH}" height="${WFH}"></canvas></div>
    </div>
    <div class="status"></div>`;
  document.getElementById('ants').appendChild(wrap);
  const cvs=wrap.querySelectorAll('canvas');
  const wf=cvs[2].getContext('2d');
  cards[ant]={mag:cvs[0].getContext('2d'),phase:cvs[1].getContext('2d'),
              wf:wf,wfimg:wf.createImageData(WFW,1),
              cons:cvs[3].getContext('2d'),
              status:wrap.querySelector('.status'),frame:0,
              el:wrap,badge:wrap.querySelector('.badge'),
              lastCsi:-1,lastCns:-1};
}

// Grid + labelled ticks. yfmt(v) renders a y value; nsc labels the x axis in
// subcarriers (DC centred). Without labels you cannot tell a rescale from real
// movement, which is exactly how an auto-ranged panel misleads.
function axes(ctx,ymin,ymax,yfmt,nsc){
  ctx.clearRect(0,0,W,H); ctx.strokeStyle='#21262d'; ctx.lineWidth=1;
  for(let i=0;i<=4;i++){const y=H*i/4|0;ctx.beginPath();ctx.moveTo(0,y+.5);ctx.lineTo(W,y+.5);ctx.stroke();}
  const xz=W/2|0; ctx.beginPath();ctx.moveTo(xz+.5,0);ctx.lineTo(xz+.5,H);ctx.stroke();
  if(!yfmt) return;
  ctx.fillStyle='#6e7681'; ctx.font='9px sans-serif'; ctx.textAlign='left';
  for(let i=0;i<=4;i++){
    const y=H*i/4, v=ymax-(ymax-ymin)*i/4;
    ctx.fillText(yfmt(v), 2, Math.min(H-2, Math.max(9, y+ (i===0?9:(i===4?-2:3)))));
  }
  if(nsc){
    ctx.textAlign='center'; ctx.fillText('DC', xz, H-2);
    ctx.textAlign='left';   ctx.fillText('-'+(nsc>>1), 20, H-2);
    ctx.textAlign='right';  ctx.fillText('+'+(nsc>>1), W-2, H-2);
    ctx.textAlign='left';
  }
}

function line(ctx,vals,ymin,ymax,color){
  const n=vals.length; ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.beginPath();
  let started=false;
  for(let k=0;k<n;k++){
    const v=vals[k];
    if(v===null){started=false;continue;}
    const x=W*k/(n-1);
    const y=H-(v-ymin)/(ymax-ymin)*H;
    if(!started){ctx.moveTo(x,y);started=true;}else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

function drawCsi(card,c){
  card.frame=c.frame;
  // Magnitude: FIXED axis, never re-ranged. Set with --mag-top / --mag-span.
  const top=MAG_TOP, bot=MAG_BOT;
  axes(card.mag,bot,top,v=>v.toFixed(0),c.sc);
  line(card.mag,c.mag_db,bot,top,'#58a6ff');
  card.mag.font='9px sans-serif'; card.mag.textAlign='right';
  // A fixed axis can hide the trace entirely if the level moves off scale, so say
  // so rather than showing an innocent-looking empty panel.
  const fin=c.mag_db.filter(v=>v!==null);
  const off=fin.length&&(Math.max(...fin)>top||Math.min(...fin)<bot);
  card.mag.fillStyle=off?'#f85149':'#6e7681';
  card.mag.fillText(off?'dB  OFF SCALE':'dB', W-2, 10);
  card.mag.textAlign='left';
  // Phase: fixed -pi..+pi (it always was), now with ticks in units of pi.
  axes(card.phase,-Math.PI,Math.PI,v=>(v/Math.PI).toFixed(1)+'\u03c0',c.sc);
  line(card.phase,c.phase,-Math.PI,Math.PI,'#3fb950');
  card.phase.fillStyle='#6e7681'; card.phase.font='9px sans-serif';
  card.phase.textAlign='right'; card.phase.fillText('rad', W-2, 10);
  card.phase.textAlign='left';
  // waterfall: scroll up 1px, draw new bottom row colored by magnitude
  card.wf.drawImage(card.wf.canvas,0,-1);
  const n=c.mag_db.length, d=card.wfimg.data;
  for(let x=0;x<WFW;x++){
    const k=Math.min(n-1,(x*n/WFW)|0), v=c.mag_db[k];
    let col=[13,15,20];
    if(v!==null){ col=jet((v-bot)/(top-bot)); }
    d[4*x]=col[0];d[4*x+1]=col[1];d[4*x+2]=col[2];d[4*x+3]=255;
  }
  card.wf.putImageData(card.wfimg,0,WFH-1);
  card.status.textContent='frame '+c.frame+' · '+c.sc+' subcarriers · '
     +(c.rate/1e6).toFixed(2)+' MS/s · peak '+c.peak_db.toFixed(1)+' dB';
}

// ideal alphabet (unit average power), mod = bits/symbol (2=QPSK,4=16QAM,6=64QAM)
function idealPts(mod){
  const L=Math.round(Math.sqrt(Math.pow(2,mod)));  // levels per dimension
  const lv=[]; for(let i=0;i<L;i++) lv.push(-(L-1)+2*i);
  let p=0; for(const a of lv)for(const b of lv) p+=a*a+b*b;
  const nrm=Math.sqrt(p/(L*L)), out=[];
  for(const a of lv)for(const b of lv) out.push([a/nrm,b/nrm]);
  return out;
}
function drawCons(card,cn){
  const ctx=card.cons, S=WFH, R=S/2/1.7;  // unit power -> R px
  ctx.clearRect(0,0,S,S);
  ctx.strokeStyle='#21262d'; ctx.lineWidth=1; ctx.beginPath();
  ctx.moveTo(S/2,0);ctx.lineTo(S/2,S);ctx.moveTo(0,S/2);ctx.lineTo(S,S/2);ctx.stroke();
  ctx.fillStyle='rgba(88,166,255,0.55)';   // received points
  for(const p of cn.pts){const x=S/2+p[0]*R,y=S/2-p[1]*R;ctx.fillRect(x,y,1.6,1.6);}
  ctx.fillStyle='#f85149';                 // ideal alphabet
  for(const p of idealPts(cn.mod)){const x=S/2+p[0]*R,y=S/2-p[1]*R;ctx.fillRect(x-2,y-2,4,4);}
}

let pktCount=0,t0=Date.now();
function onData(obj){
  const ant=obj.ant;
  for(const a in ant){
    if(!cards[a]) makeCard(a);
    const rec=ant[a], card=cards[a];
    // A stale re-push carries the SAME record, so gate redraw and the rate meter
    // on the frame number. Otherwise a stalled link would read as busy.
    if(rec.csi && rec.csi.frame!==card.lastCsi){
      drawCsi(card,rec.csi); card.lastCsi=rec.csi.frame; pktCount++;
    }
    if(rec.cns && rec.cns.frame!==card.lastCns){
      drawCons(card,rec.cns); card.lastCns=rec.cns.frame;
    }
    // The sounder stops sending for an antenna whose slots carried RX gaps, so
    // the panels hold their last good estimate. Say that on screen: a frozen
    // panel and a healthy static channel look identical otherwise.
    const age=rec.age_ms||0;
    if(age>=STALE_MS){
      card.el.classList.add('stale');
      card.badge.textContent='stale '+(age/1000).toFixed(1)+' s';
    }else{
      card.el.classList.remove('stale');
    }
  }
  const dt=(Date.now()-t0)/1000;
  document.getElementById('meta').textContent=
    Object.keys(ant).length+' antenna(s) · '+(pktCount/dt).toFixed(0)+' updates/s';
}

function connect(){
  const es=new EventSource('/stream');
  es.onmessage=e=>{ try{onData(JSON.parse(e.data));}catch(err){} };
  es.onerror=()=>{ document.getElementById('meta').innerHTML=
     '<span class="off">disconnected — retrying…</span>'; };
}
connect();
</script>
</body></html>
"""

if __name__ == "__main__":
    main()
