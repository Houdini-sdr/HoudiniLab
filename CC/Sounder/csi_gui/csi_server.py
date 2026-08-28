#!/usr/bin/env python3
"""Live CSI web dashboard for the RENEW/Houdini sounder.

Runs the sounder in *viewing mode* (``sounder --view``), which computes per-antenna
CSI in C++ (pilot-agnostic -- uses the config's freq-domain reference, so LTS /
Zadoff-Chu / any ``pilot_seq`` works) and streams one UDP datagram per (frame,
antenna). This backend receives those datagrams, and serves a self-contained web
page (Server-Sent Events + HTML5 canvas) that shows, per RX antenna, live magnitude
& phase across subcarriers, a scrolling waterfall, and the equalized constellation.
Scales automatically to however many antennas appear in the stream.

The page is styled with Tabler, vendored at ``csi_gui/vendor/tabler.min.css`` and
served from this process, so the dashboard matches the RayNet compiler dashboard
and still needs nothing installed: Python's standard library plus one CSS file
that ships in the repo.

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
        elif self.path.startswith("/vendor/"):
            self._static(self.path[len("/vendor/"):].split("?", 1)[0])
        elif self.path.startswith("/stream"):
            self._sse()
        else:
            self.send_error(404)

    def _static(self, name):
        """Serve one vendored asset out of ``csi_gui/vendor``.

        The allow-list is the whole access control: the page is the only client and
        it asks for exactly one file, so there is no reason to let a request name a
        path at all, let alone walk out of the directory.
        """
        mime = {"tabler.min.css": "text/css; charset=utf-8"}.get(name)
        if mime is None:
            self.send_error(404)
            return
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vendor", name)
        try:
            with open(path, "rb") as f:
                body = f.read()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        # Vendored with the repo, so it cannot change under a running server: let
        # the browser keep it rather than refetch half a megabyte on every reload.
        self.send_header("Cache-Control", "public, max-age=86400")
        self.end_headers()
        self.wfile.write(body)

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
<html lang="en" data-bs-theme="dark">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Houdini live CSI</title>
<link rel="stylesheet" href="/vendor/tabler.min.css">
<style>
/* Local layer: only what Tabler has no class for. Every colour here is a Tabler
   variable, so light and dark both follow the theme and there is no second palette
   to keep in step. Same reason the canvases read their colours from these vars. */
.csi-cards{display:flex;flex-wrap:wrap;gap:1rem;padding:1rem;align-items:flex-start}
.csi-card{width:620px}
/* A stale card dims its plots but NOT its header, so the badge that explains the
   dimming does not dim along with the thing it is explaining. */
.csi-card.stale .csi-plots{opacity:.4}
.csi-plots{display:grid;grid-template-columns:1fr 1fr;gap:.75rem 1rem}
.csi-plot-title{font-size:.7rem;color:var(--tblr-secondary);margin-bottom:.15rem;
  display:flex;align-items:center;gap:.35rem}
.csi-stage{display:flex}
.csi-y-axis{position:relative;width:36px;flex:0 0 36px;font-size:.65rem;
  color:var(--tblr-secondary);font-variant-numeric:tabular-nums}
.csi-y-axis span{position:absolute;right:5px;transform:translateY(-50%);white-space:nowrap}
/* The top and bottom labels sit ON the canvas edge, so centring them there would
   clip half of each. Align them inward instead, which is what raynet's YAxis does
   with its per-tick pixel nudge. */
.csi-y-axis span:first-child{transform:translateY(0)}
.csi-y-axis span:last-child{transform:translateY(-100%)}
.csi-plot canvas{display:block;background:var(--tblr-bg-surface-tertiary);
  border:1px solid var(--tblr-border-color);border-radius:4px}
.csi-x-axis{display:flex;justify-content:space-between;margin-left:36px;margin-top:.1rem;
  font-size:.65rem;color:var(--tblr-secondary);font-variant-numeric:tabular-nums}
.tnum{font-variant-numeric:tabular-nums}
</style>
</head>
<body>
<header class="navbar navbar-expand-md d-print-none">
  <div class="container-fluid">
    <span class="navbar-brand d-flex align-items-center gap-2 fw-bold mb-0">
      <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24"
           fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"
           stroke-linejoin="round" class="icon"><path stroke="none" d="M0 0h24v24H0z" fill="none"/>
        <path d="M6 18l0 -3"/><path d="M10 18l0 -6"/><path d="M14 18l0 -9"/><path d="M18 18l0 -12"/></svg>
      <span>Houdini live CSI</span>
    </span>
    <div class="ms-auto d-flex align-items-center gap-3">
      <span class="text-secondary tnum" id="meta">connecting&hellip;</span>
      <button class="btn btn-icon btn-ghost-secondary" id="theme"
              title="Toggle light / dark" aria-label="Toggle light / dark"></button>
    </div>
  </div>
</header>
<div class="csi-cards" id="ants"></div>
<script>
const W=250,H=120,WFW=250,WFH=120;   // plot sizes
const STALE_MS=__STALE_MS__;         // no update for this long -> dim + badge
// Both top panels are FIXED frame to frame. An axis that re-ranges per frame makes
// a static channel look alive and hides real drift, so nothing here auto-scales.
// (raynet-compiler's LinePlot deliberately does re-range: that is right for a
// reviewed capture and wrong for a live one. Do not copy it here.)
const MAG_TOP=__MAG_TOP__, MAG_BOT=__MAG_TOP__-__MAG_SPAN__;
const CONS_R=1.7;                     // constellation half-width, in unit-power units
const cards={};                       // ant_id -> {mag,phase,wf,wfimg,cons,...}

// ---- theme ---------------------------------------------------------------
// Same mechanism as the RayNet dashboard: the theme is one attribute on <html>,
// Tabler's variables do the rest, and localStorage remembers the choice. The
// markup above declares the fallback, exactly as their index.html does.
const THEME_KEY='houdini-csi-theme';
const ICON='<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24"'
  +' fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"'
  +' stroke-linejoin="round" class="icon"><path stroke="none" d="M0 0h24v24H0z" fill="none"/>';
const SUN=ICON+'<path d="M12 12m-4 0a4 4 0 1 0 8 0a4 4 0 1 0 -8 0"/><path d="M3 12h1m8 -9v1m8 8h1'
  +'m-9 8v1m-6.4 -15.4l.7 .7m12.1 -.7l-.7 .7m0 11.4l.7 .7m-12.1 -.7l-.7 .7"/></svg>';
const MOON=ICON+'<path d="M12 3c.132 0 .263 0 .393 0a7.5 7.5 0 0 0 7.92 12.446a9 9 0 1 1 -8.313'
  +' -12.454z"/></svg>';

// A canvas gets no CSS, so read the theme's colours out of Tabler once per change
// and hand them to the draw functions. This is the only place the page names a
// colour at all, which is what keeps the plots and the chrome in step.
let C={};
function readTheme(){
  const s=getComputedStyle(document.documentElement), v=n=>s.getPropertyValue(n).trim();
  C={grid:v('--tblr-border-color'), bg:v('--tblr-bg-surface-tertiary'),
     mag:v('--tblr-azure'), phase:v('--tblr-green'), warn:v('--tblr-red'),
     muted:v('--tblr-secondary'),
     pts:'rgba('+v('--tblr-azure-rgb')+',0.55)'};
}
function applyTheme(t){
  document.documentElement.setAttribute('data-bs-theme',t);
  try{ localStorage.setItem(THEME_KEY,t); }catch(e){}
  document.getElementById('theme').innerHTML=(t==='dark'?SUN:MOON);
  readTheme(); redrawAll();
}
function initTheme(){
  let t=null;
  try{ t=localStorage.getItem(THEME_KEY); }catch(e){}
  if(t!=='light'&&t!=='dark') t=document.documentElement.getAttribute('data-bs-theme')||'dark';
  applyTheme(t);
  document.getElementById('theme').addEventListener('click',()=>applyTheme(
    document.documentElement.getAttribute('data-bs-theme')==='dark'?'light':'dark'));
}

// ---- formatting ----------------------------------------------------------
// Ported from raynet-compiler src/dashboard/components/plots.tsx so a number reads
// the same in both tools.
function formatAxisValue(v){
  if(!Number.isFinite(v)) return 'n/a';
  const a=Math.abs(v);
  if(a!==0&&(a>=10000||a<0.01)) return v.toExponential(1);
  if(a>=100) return v.toFixed(0);
  if(a>=10) return v.toFixed(1);
  return v.toFixed(2);
}
function formatScaled(v,mode){
  if(!Number.isFinite(v)) return 'n/a';
  if(mode==='db') return v.toFixed(1)+' dB';
  const a=Math.abs(v);
  if(a!==0&&(a>=10000||a<0.001)) return v.toExponential(2);
  return v.toFixed(a>=100?1:4);
}

function jet(v){ // v in [0,1] -> [r,g,b]
  v=Math.max(0,Math.min(1,v));
  const r=Math.max(0,Math.min(1,1.5-Math.abs(4*v-3)));
  const g=Math.max(0,Math.min(1,1.5-Math.abs(4*v-2)));
  const b=Math.max(0,Math.min(1,1.5-Math.abs(4*v-1)));
  return [r*255|0,g*255|0,b*255|0];
}

// ---- card construction ---------------------------------------------------
// Tick labels live in HTML gutters beside the canvas rather than inside it: the
// trace gets the whole canvas back, and the labels pick up the theme's text colour
// for free. They are written ONCE because these axes never move -- a label that
// never changes is the honest rendering of an axis that never re-ranges.
function yAxis(labels){   // labels top -> bottom
  return '<div class="csi-y-axis">'+labels.map((t,i)=>
    '<span style="top:'+(i/(labels.length-1)*100).toFixed(1)+'%">'+t+'</span>').join('')+'</div>';
}
function frame(title, w, h, ylabels, xlabels, extra){
  return '<div class="csi-plot"><div class="csi-plot-title"><span>'+title+'</span>'
    +(extra||'')+'</div><div class="csi-stage">'+yAxis(ylabels)
    +'<canvas width="'+w+'" height="'+h+'"></canvas></div>'
    +'<div class="csi-x-axis">'+xlabels.map(t=>'<span>'+t+'</span>').join('')+'</div></div>';
}

function magLabels(){
  const out=[];
  for(let i=0;i<=4;i++) out.push(formatAxisValue(MAG_TOP-(MAG_TOP-MAG_BOT)*i/4));
  return out;
}
function makeCard(ant){
  const wrap=document.createElement('div');
  wrap.className='card csi-card';
  const off='<span class="badge bg-red-lt text-red csi-off" hidden>off scale</span>';
  wrap.innerHTML=
    '<div class="card-header py-2">'
     +'<h3 class="card-title">RX antenna '+ant+'</h3>'
     +'<div class="card-actions"><span class="badge bg-orange-lt text-orange csi-stale" hidden></span></div>'
    +'</div>'
    +'<div class="card-body p-3">'
     +'<div class="csi-plots">'
      +frame('|H| (dB) vs subcarrier',W,H,magLabels(),['','',''],off)
      +frame('phase (rad)',W,H,['1.0π','0.5π','0.0π','-0.5π','-1.0π'],['','',''])
      +frame('waterfall |H| (time down)',WFW,WFH,['older','','now'],['','',''])
      +frame('constellation (equalized U)',WFH,WFH,
             [formatAxisValue(CONS_R),'0.00',formatAxisValue(-CONS_R)],
             [formatAxisValue(-CONS_R),'I','+'+formatAxisValue(CONS_R)])
     +'</div>'
     +'<div class="text-secondary tnum mt-3 csi-status" style="font-size:.75rem"></div>'
    +'</div>';
  document.getElementById('ants').appendChild(wrap);
  const cvs=wrap.querySelectorAll('canvas'), wf=cvs[2].getContext('2d');
  cards[ant]={mag:cvs[0].getContext('2d'),phase:cvs[1].getContext('2d'),
              wf:wf,wfimg:wf.createImageData(WFW,1),cons:cvs[3].getContext('2d'),
              status:wrap.querySelector('.csi-status'),
              el:wrap,badge:wrap.querySelector('.csi-stale'),
              off:wrap.querySelector('.csi-off'),
              xax:wrap.querySelectorAll('.csi-x-axis'),
              lastCsi:-1,lastCns:-1,csiRec:null,cnsRec:null,frame:0};
}

// Grid only: the labels are HTML now. Horizontal quarters plus the DC centre line.
function grid(ctx,w,h){
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle=C.grid; ctx.lineWidth=1;
  for(let i=0;i<=4;i++){const y=(h*i/4)|0;
    ctx.beginPath();ctx.moveTo(0,y+.5);ctx.lineTo(w,y+.5);ctx.stroke();}
  const xz=(w/2)|0; ctx.beginPath();ctx.moveTo(xz+.5,0);ctx.lineTo(xz+.5,h);ctx.stroke();
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

// The subcarrier axis is only known once a frame has arrived, so the three x-axis
// gutters that share it are filled in on the first one and left alone after.
function setScAxis(card,nsc){
  if(card.nsc===nsc) return;
  card.nsc=nsc;
  const lab=['-'+(nsc>>1),'DC','+'+(nsc>>1)];
  for(let i=0;i<3;i++){
    const sp=card.xax[i].querySelectorAll('span');
    for(let j=0;j<3;j++) sp[j].textContent=lab[j];
  }
}

function drawCsi(card,c,advance){
  card.frame=c.frame; card.csiRec=c;
  setScAxis(card,c.sc);
  // Magnitude: FIXED axis, never re-ranged. Set with --mag-top / --mag-span.
  const top=MAG_TOP, bot=MAG_BOT;
  grid(card.mag,W,H);
  line(card.mag,c.mag_db,bot,top,C.mag);
  // A fixed axis can hide the trace entirely if the level moves off scale, so say
  // so rather than showing an innocent-looking empty panel.
  const fin=c.mag_db.filter(v=>v!==null);
  card.off.hidden=!(fin.length&&(Math.max(...fin)>top||Math.min(...fin)<bot));
  // Phase: fixed -pi..+pi (it always was).
  grid(card.phase,W,H);
  line(card.phase,c.phase,-Math.PI,Math.PI,C.phase);
  // waterfall: scroll up 1px, draw new bottom row coloured by magnitude
  if(advance!==false){
    card.wf.drawImage(card.wf.canvas,0,-1);
    const n=c.mag_db.length, d=card.wfimg.data;
    for(let x=0;x<WFW;x++){
      const k=Math.min(n-1,(x*n/WFW)|0), v=c.mag_db[k];
      let col=[0,0,0];
      if(v!==null) col=jet((v-bot)/(top-bot));
      d[4*x]=col[0];d[4*x+1]=col[1];d[4*x+2]=col[2];d[4*x+3]=255;
    }
    card.wf.putImageData(card.wfimg,0,WFH-1);
  }
  card.status.textContent='frame '+c.frame+' · '+c.sc+' subcarriers · '
     +(c.rate/1e6).toFixed(2)+' MS/s · peak '+formatScaled(c.peak_db,'db');
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
  card.cnsRec=cn;
  const ctx=card.cons, S=WFH, R=S/2/CONS_R;  // unit power -> R px
  ctx.clearRect(0,0,S,S);
  ctx.strokeStyle=C.grid; ctx.lineWidth=1; ctx.beginPath();
  ctx.moveTo(S/2,0);ctx.lineTo(S/2,S);ctx.moveTo(0,S/2);ctx.lineTo(S,S/2);ctx.stroke();
  ctx.fillStyle=C.pts;                     // received points
  for(const p of cn.pts){const x=S/2+p[0]*R,y=S/2-p[1]*R;ctx.fillRect(x,y,1.6,1.6);}
  ctx.fillStyle=C.warn;                    // ideal alphabet
  for(const p of idealPts(cn.mod)){const x=S/2+p[0]*R,y=S/2-p[1]*R;ctx.fillRect(x-2,y-2,4,4);}
}

// A theme change has to repaint every canvas from the last record, because a
// stalled stream will not repaint them for us. The waterfall history lives in the
// bitmap and cannot be recoloured, so it is cleared to the new background rather
// than left as a rectangle of the old theme.
function redrawAll(){
  for(const a in cards){
    const card=cards[a];
    card.wf.fillStyle=C.bg; card.wf.fillRect(0,0,WFW,WFH);
    if(card.csiRec) drawCsi(card,card.csiRec,false);
    if(card.cnsRec) drawCons(card,card.cnsRec);
  }
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
      drawCsi(card,rec.csi,true); card.lastCsi=rec.csi.frame; pktCount++;
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
      card.badge.hidden=false;
      card.badge.textContent='stale '+(age/1000).toFixed(1)+' s';
    }else{
      card.el.classList.remove('stale');
      card.badge.hidden=true;
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
     '<span class="text-red">disconnected, retrying&hellip;</span>'; };
}
initTheme();
connect();
</script>
</body></html>
"""

if __name__ == "__main__":
    main()
