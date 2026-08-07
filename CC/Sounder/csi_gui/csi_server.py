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

MAGIC = 0x43534931  # "CSI1"
HDR = struct.Struct("<IIIIf")  # magic, frame, ant, num_sc, rate

# ---- shared state: latest CSI per antenna ----------------------------------
_lock = threading.Lock()
_latest = {}   # ant_id -> dict(frame, sc, rate, mag_db=[...|None], phase=[...|None])
_seq = 0       # bumps on every new datagram so the SSE loop knows there's fresh data
_stats = {"pkts": 0, "t0": time.time()}


def _to_csi(payload):
    """Parse one datagram body into per-subcarrier magnitude(dB) and phase(rad)."""
    magic, frame, ant, nsc, rate = HDR.unpack_from(payload, 0)
    if magic != MAGIC:
        return None
    off = HDR.size
    need = off + 8 * nsc
    if len(payload) < need:
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
    return {"frame": int(frame), "ant": int(ant), "sc": int(nsc),
            "rate": float(rate), "mag_db": mag_db, "phase": phase,
            "peak_db": (20.0 * math.log10(peak) if peak > 0 else 0.0)}


def _udp_loop(bind_host, bind_port):
    global _seq
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((bind_host, bind_port))
    print("[csi] listening for CSI on %s:%d" % (bind_host, bind_port), flush=True)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except OSError:
            break
        csi = _to_csi(data)
        if csi is None:
            continue
        with _lock:
            _latest[csi["ant"]] = csi
            _seq += 1
            _stats["pkts"] += 1


def _snapshot():
    with _lock:
        return _seq, {str(a): _latest[a] for a in _latest}


# ---- HTTP / SSE ------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):  # quiet
        pass

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = PAGE.encode("utf-8")
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
        last_seq = -1
        try:
            while True:
                seq, snap = _snapshot()
                if seq != last_seq and snap:
                    last_seq = seq
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
def _launch_sounder(args, udp_dest):
    """Run the sounder in viewing mode on this host (teardown + env + retries)."""
    env = os.environ.copy()
    env["HOUDINI_CSI_UDP"] = udp_dest
    env["HOUDINI_MAX_FRAME"] = str(args.max_frame)
    if args.csi_fps:
        env["HOUDINI_CSI_FPS"] = str(args.csi_fps)
    sd = args.sounder_dir
    # A tiny shell wrapper: teardown any stuck framer, then run sounder --view,
    # retrying the flaky cold-start. Mirrors the HIL test harness.
    script = (
        'source ~/houdini_test/bin/activate 2>/dev/null; '
        'export LD_LIBRARY_PATH=$HOME/houdini_test/lib '
        'SOAPY_SDR_PLUGIN_PATH=$HOME/houdini_test/lib/SoapySDR/modules0.8-3; '
        'cd "%s"; '
        'for a in 1 2 3 4; do '
        '  timeout 60 python3 /tmp/td.py >/dev/null 2>&1; sleep 8; '
        '  ./build/sounder --view --conf_file "%s" --storepath "%s" 2>&1 | '
        '     sed -u "s/^/[sounder] /"; '
        '  echo "[sounder] exited, retrying..."; sleep 5; '
        'done'
    ) % (sd, args.conf, args.storepath)
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
    ap.add_argument("--launch", action="store_true",
                    help="also launch the sounder in viewing mode on this host")
    ap.add_argument("--sounder-dir", default=os.path.expanduser("~/repos/HoudiniLab/CC/Sounder"))
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
    srv.daemon_threads = True
    url = "http://localhost:%d/" % args.http_port
    print("[csi] dashboard at %s  (SSH: -L %d:localhost:%d)"
          % (url, args.http_port, args.http_port), flush=True)

    def _shutdown(*_):
        print("\n[csi] shutting down", flush=True)
        if child is not None:
            try:
                os.killpg(os.getpgid(child.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
        srv.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    srv.serve_forever()


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
  wrap.innerHTML=`<h2>RX antenna ${ant}</h2>
    <div class="row">
      <div class="plot"><span class="lbl">|H| (dB) vs subcarrier</span>
        <canvas width="${W}" height="${H}"></canvas></div>
      <div class="plot"><span class="lbl">phase (rad)</span>
        <canvas width="${W}" height="${H}"></canvas></div>
    </div>
    <div class="row" style="margin-top:8px">
      <div class="plot"><span class="lbl">waterfall |H| (time ↓)</span>
        <canvas width="${WFW}" height="${WFH}"></canvas></div>
      <div class="status" style="flex:1"></div>
    </div>`;
  document.getElementById('ants').appendChild(wrap);
  const cvs=wrap.querySelectorAll('canvas');
  const wf=cvs[2].getContext('2d');
  const img=wf.createImageData(WFW,1);
  cards[ant]={mag:cvs[0].getContext('2d'),phase:cvs[1].getContext('2d'),
              wf:wf,wfimg:img,status:wrap.querySelector('.status'),frame:0};
}

function axes(ctx,ymin,ymax,label){
  ctx.clearRect(0,0,W,H); ctx.strokeStyle='#21262d'; ctx.lineWidth=1;
  for(let i=0;i<=4;i++){const y=H*i/4|0;ctx.beginPath();ctx.moveTo(0,y+.5);ctx.lineTo(W,y+.5);ctx.stroke();}
  const xz=W/2|0; ctx.beginPath();ctx.moveTo(xz+.5,0);ctx.lineTo(xz+.5,H);ctx.stroke();
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

function draw(ant,c){
  const card=cards[ant]; if(!card) return;
  card.frame=c.frame;
  // magnitude, auto-ranged to [peak-40, peak+3] dB
  const top=c.peak_db+3, bot=c.peak_db-40;
  axes(card.mag); line(card.mag,c.mag_db,bot,top,'#58a6ff');
  card.mag.fillStyle='#8b949e';card.mag.font='9px sans-serif';
  card.mag.fillText(top.toFixed(0)+'dB',2,10);card.mag.fillText(bot.toFixed(0),2,H-2);
  // phase
  axes(card.phase); line(card.phase,c.phase,-Math.PI,Math.PI,'#3fb950');
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

let lastFrames={},pktCount=0,t0=Date.now();
function onData(obj){
  const ant=obj.ant;
  for(const a in ant){
    if(!cards[a]) makeCard(a);
    draw(a,ant[a]); pktCount++;
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
