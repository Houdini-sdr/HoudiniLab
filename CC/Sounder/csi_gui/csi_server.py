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

Wire formats (little-endian), one datagram per (frame, antenna) per kind:

  CSI2  [magic][frame][ant][num_sc][rate f32][reps]  then num_sc * (H_re f32, H_im f32)
        then num_sc * (raw phase f32): arg(H) in [-pi, pi] BEFORE the display
        de-ramp and the per-run anchor. (Historically this block carried the
        repeat coherence; `reps` is now a constant 1 kept for layout.) CSI1 is
        the same without [reps] and without the trailing block, and is still
        accepted.
  CNS1  [magic][frame][ant][num_pts][mod_order]      then num_pts * (I f32, Q f32)
  SYN1  [magic][frame][tid][state][resid i32][cfo_hz f32][snr f32][shift i32]
        [samps_per_frame u32][carrier_hz f32][scatter_tol u32]
        One datagram per resync DETECTION from the UE sync thread (not the
        recording path), so it is NOT per-antenna and carries no payload array.
        state: 1=LOCKED (beacon alive on the anchored grid), 2=HOLD (one
        off-grid detection seen, deliberately NOT acted on), 3=ESCALATING (the
        anchor was re-acquired; `shift` is the schedule step applied, already
        reduced modulo the frame period and centred), 4=WEAK (detected but under
        the SNR floor -- distinct from no beacon at all), 5=REANCHOR FAILED (an
        escalation ran and re-acquisition did not confirm). NOT SYNCED is inferred
        from staleness -- while the acquisition loop hunts there is no detection
        to hang a datagram on.
  ADC2  [magic][frame][ant][cols][samps][rate f32][peak][clipped][slot][any_peak]
        [any_clipped]  then cols * (I_min, I_max, Q_min, Q_max) as int16.  A min/max
        envelope of the whole slot rather than decimated samples, so a brief clip
        cannot fall between two plotted points.  The envelope is the PILOT slot only,
        because a frame's slots differ in level by orders of magnitude and mixing
        them makes every update a different signal; peak/clipped describe that slot,
        any_peak/any_clipped cover every slot since the previous send.  ADC1 is the
        same without the last three fields and is still accepted.
"""
import argparse
import collections
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

MAGIC_CSI = 0x43534931   # "CSI1" -- pilot channel estimate (legacy, no quality)
MAGIC_CSI2 = 0x43534932  # "CSI2" -- as CSI1 plus per-subcarrier raw phase
MAGIC_CNS = 0x434E5331   # "CNS1" -- equalized uplink-data constellation
MAGIC_ADC = 0x41444331   # "ADC1" -- raw-ADC envelope, any slot (legacy)
MAGIC_ADC2 = 0x41444332  # "ADC2" -- pilot-slot envelope + an all-slot clip ledger
MAGIC_SYN = 0x53594E31   # "SYN1" -- UE beacon sync state, resid and CFO
CSI_HDR = struct.Struct("<IIIIf")   # magic, frame, ant, num_sc, rate
CSI2_HDR = struct.Struct("<IIIIfI")  # ... plus reps (pilot symbols averaged)
CNS_HDR = struct.Struct("<IIIII")   # magic, frame, ant, num_pts, mod_order
ADC_HDR = struct.Struct("<IIIIIfII")   # magic, frame, ant, cols, samps, rate, peak, clipped
ADC2_HDR = struct.Struct("<IIIIIfIIIII")  # ... plus slot, any_peak, any_clipped
SYN_HDR = struct.Struct("<IIIIiffiIfI")  # ... plus samps_per_frame, carrier_hz, scatter_tol

# ---- shared state: latest CSI + constellation per antenna ------------------
_lock = threading.Lock()
_latest = {}   # ant_id -> {"csi": {...}, "cns": {...}}
_seq = 0       # bumps on every new datagram so the SSE loop knows there's fresh data
_stats = {"pkts": 0, "t0": time.time()}
# Sync/CFO history is a TIME SERIES, not a latest-value, so it lives beside the
# per-antenna state rather than in it. maxlen caps memory on a long run; the page
# shows a shorter window than this.
SYNC_KEEP = 240
# Keyed by tid, which the wire format carries precisely to identify the source;
# a single global deque would interleave two clients into one unreadable trace.
_sync = {}     # tid -> deque of records
_sync_t = {}   # tid -> monotonic time of that tid's last SYN1
_sync_bad = [0]     # datagrams rejected as non-finite
_bad_payload = [0]  # SSE snapshots dropped because a float would not serialise
SYNC_REPUSH_CEIL_MS = 120000  # past this a quiet tid stops driving re-pushes
# tid arrives as a uint32 off the wire, so the map is unbounded by construction.
# Entries are NOT pruned on age -- the record is what lets the page show
# NOT SYNCED for a link that stopped -- so cap the count instead. A sounder runs
# a handful of clients; anything past this is a malformed or hostile datagram.
MAX_SYNC_TIDS = 8


def _parse_csi(payload, with_quality):
    """CSI1 or CSI2 -> one antenna's channel estimate.

    Both layouts are accepted so a dashboard running ahead of an un-rebuilt sounder
    still draws everything except the raw-phase panel, rather than drawing nothing.
    """
    if with_quality:
        magic, frame, ant, nsc, rate, reps = CSI2_HDR.unpack_from(payload, 0)
        off = CSI2_HDR.size
        need = off + 12 * nsc
    else:
        magic, frame, ant, nsc, rate = CSI_HDR.unpack_from(payload, 0)
        off, reps = CSI_HDR.size, 0
        need = off + 8 * nsc
    if len(payload) < need:
        return None
    vals = struct.unpack_from("<%df" % (2 * nsc), payload, off)
    # CSI2's trailing float block carries the RAW per-subcarrier phase
    # (radians): arg(H) before the display de-ramp and the per-run anchor.
    rawph_in = (struct.unpack_from("<%df" % nsc, payload, off + 8 * nsc)
                if with_quality else None)
    mag_db, phase, mags, rawph = [], [], [], []
    for k in range(nsc):
        re, im = vals[2 * k], vals[2 * k + 1]
        m = math.hypot(re, im)
        mags.append(m)
        if m < 1e-9:               # unused subcarrier (guard band / DC null)
            mag_db.append(None)
            phase.append(None)
            rawph.append(None)     # a gap, not a zero: nothing was measured here
        else:
            mag_db.append(20.0 * math.log10(m))
            phase.append(math.atan2(im, re))
            rawph.append(rawph_in[k] if rawph_in else None)
    peak = max((m for m in mags if m > 0), default=0.0)
    return int(ant), {"frame": int(frame), "sc": int(nsc), "rate": float(rate),
                      "mag_db": mag_db, "phase": phase,
                      "raw_ph": (rawph if with_quality else None),
                      "peak_db": (20.0 * math.log10(peak) if peak > 0 else 0.0)}


def _parse_adc(payload, v2):
    """ADC1/ADC2 -> one antenna's raw-sample min/max envelope plus clip counts.

    ADC2 draws the PILOT slot only and carries a separate ledger covering every slot
    seen since the previous send. A frame's slots differ in level by orders of
    magnitude, so mixing them into one panel made every update a different signal;
    the ledger is how clipping on an undrawn slot still gets reported.
    """
    if v2:
        (magic, frame, ant, cols, samps, rate, peak, clipped,
         slot, any_peak, any_clipped) = ADC2_HDR.unpack_from(payload, 0)
        off = ADC2_HDR.size
    else:
        magic, frame, ant, cols, samps, rate, peak, clipped = ADC_HDR.unpack_from(payload, 0)
        off = ADC_HDR.size
        slot, any_peak, any_clipped = -1, peak, clipped
    if len(payload) < off + 8 * cols:
        return None
    e = struct.unpack_from("<%dh" % (4 * cols), payload, off)
    return int(ant), {"frame": int(frame), "cols": int(cols), "samps": int(samps),
                      "rate": float(rate), "peak": int(peak), "clipped": int(clipped),
                      "slot": int(slot), "any_peak": int(any_peak),
                      "any_clipped": int(any_clipped),
                      "i_min": e[0::4], "i_max": e[1::4],
                      "q_min": e[2::4], "q_max": e[3::4],
                      "full_scale": 32767}


def _parse_cns(payload):
    magic, frame, ant, npt, mod = CNS_HDR.unpack_from(payload, 0)
    off = CNS_HDR.size
    if len(payload) < off + 8 * npt:
        return None
    vals = struct.unpack_from("<%df" % (2 * npt), payload, off)
    pts = [[vals[2 * i], vals[2 * i + 1]] for i in range(npt)]
    return int(ant), {"frame": int(frame), "mod": int(mod), "pts": pts}


def _parse_syn(payload):
    if len(payload) != SYN_HDR.size:
        # Deliberately NOT back-compatible: the wire may change freely, but a
        # mismatch must be diagnosable from the log rather than from a packet
        # capture. Silently dropping every datagram looks exactly like "the UE
        # never locked", which is the most expensive wrong conclusion here.
        _sync_bad[0] += 1
        if _sync_bad[0] == 1 or _sync_bad[0] % 1000 == 0:
            print("[csi] SYN1 is %d bytes, this dashboard expects %d -- the "
                  "sounder is a DIFFERENT BUILD; rebuild it. %d dropped so far."
                  % (len(payload), SYN_HDR.size, _sync_bad[0]), flush=True)
        return None
    (_m, frame, tid, state, resid, cfo, snr, shift, sfr, carrier,
     tol) = SYN_HDR.unpack_from(payload, 0)
    # A NaN or inf float would serialise as bare NaN/Infinity, which is invalid
    # JSON: the browser's JSON.parse throws, onData's catch swallows it, and the
    # dashboard silently stops updating for as long as the record stays in the
    # history. Drop the datagram instead of admitting it.
    if not (math.isfinite(cfo) and math.isfinite(snr) and math.isfinite(carrier)):
        _sync_bad[0] += 1
        return None
    return {"frame": int(frame), "tid": int(tid), "state": int(state),
            "resid": int(resid), "cfo": float(cfo), "snr": float(snr),
            "shift": int(shift), "sfr": int(sfr), "fc": float(carrier),
            "tol": int(tol)}


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
            parsed, kind = _parse_csi(data, False), "csi"
        elif magic == MAGIC_CSI2:
            parsed, kind = _parse_csi(data, True), "csi"
        elif magic == MAGIC_CNS:
            parsed, kind = _parse_cns(data), "cns"
        elif magic == MAGIC_ADC:
            parsed, kind = _parse_adc(data, False), "adc"
        elif magic == MAGIC_ADC2:
            parsed, kind = _parse_adc(data, True), "adc"
        elif magic == MAGIC_SYN:
            rec = _parse_syn(data)
            if rec is not None:
                with _lock:
                    t = rec["tid"]
                    if t not in _sync and len(_sync) >= MAX_SYNC_TIDS:
                        _sync_bad[0] += 1
                        continue
                    _sync.setdefault(
                        t, collections.deque(maxlen=SYNC_KEEP)).append(rec)
                    _sync_t[t] = time.monotonic()
                    _seq += 1
                    _stats["pkts"] += 1
            continue
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
        sync = {}
        for t, hist in _sync.items():
            last = _sync_t.get(t)
            sync[str(t)] = {
                "hist": list(hist),
                "age_ms": int(max(0.0, now - last) * 1000) if last else None,
            }
        return _seq, out, sync


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
                        .replace("__GUARD_PRE__", str(self.server.guard_pre))
                        .replace("__GUARD_POST__", str(self.server.guard_post))
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
                seq, snap, sync = _snapshot()
                # While the stream is stalled _seq never moves, so pushing only on
                # a seq change would freeze the age on screen too and the badge
                # would never appear. Re-send slowly whenever anything is stale.
                now = time.monotonic()
                stale = any(r.get("age_ms", 0) >= stale_ms for r in snap.values())
                # The sync stream must count too: it is quiet by nature between
                # resync bursts, and after the staleness fix its age is the only
                # thing that can move the chip off a stale state. Without this
                # the push stops when SYN1 stops and the age freezes on screen.
                # Bounded above as well as below: past SYNC_REPUSH_CEIL_MS the
                # page has long since shown NOT SYNCED, and a tid that never
                # comes back would otherwise pin this full-payload re-push on
                # for the life of the process.
                stale = stale or any(
                    stale_ms <= (v.get("age_ms") or 0) < SYNC_REPUSH_CEIL_MS
                    for v in sync.values())
                body = None
                if (snap or sync) and (seq != last_seq or
                             (stale and now - last_stale_push >= 0.5)):
                    # allow_nan=False turns a poisoned value into an exception
                    # here rather than invalid JSON on the wire. Serialise
                    # BEFORE booking the snapshot as delivered, and fall through
                    # to the keepalive on failure -- an early `continue` here
                    # skipped the throttle at the bottom of the loop and spun
                    # the thread at 100% CPU against the UDP receiver.
                    try:
                        body = json.dumps({"ant": snap, "sync": sync},
                                          allow_nan=False)
                    except ValueError:
                        _bad_payload[0] += 1
                        body = None
                if body is not None:
                    last_seq = seq
                    last_stale_push = now
                    msg = "data: %s\n\n" % body
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
    # Validate BEFORE anything launches: a SystemExit after _launch_sounder
    # orphaned the sounder group holding the radios (second review 2.5).
    if not (math.isfinite(args.mag_top) and math.isfinite(args.mag_span)
            and args.mag_span > 0):
        raise SystemExit("--mag-top/--mag-span must be finite (span > 0)")

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
            # Surface the drop counters: both were write-only, so the two
            # failure modes they represent (a mismatched sounder build, and a
            # payload that would not serialise) were invisible in the log.
            extra = ""
            if _sync_bad[0]:
                extra += ", SYN1 dropped=%d" % _sync_bad[0]
            if _bad_payload[0]:
                extra += ", payloads dropped=%d" % _bad_payload[0]
            print("[csi] %d datagrams, antennas=%s%s" % (n, ants, extra),
                  flush=True)
    threading.Thread(target=_stats_loop, daemon=True).start()

    srv = ThreadingHTTPServer((args.http_host, args.http_port), Handler)
    srv.fps = args.fps
    srv.stale_ms = args.stale_ms
    srv.mag_top = args.mag_top
    srv.mag_span = args.mag_span
    # Nominal guard seats for the ADC panel's dashed markers, read from the
    # config when one is given (Opus review M16: 128 was hardcoded but eight
    # shipped configs use 160); harmless default otherwise.
    srv.guard_pre, srv.guard_post = 128, 128
    if getattr(args, "conf", None):
        try:
            with open(args.conf) as cf:
                cj = json.load(cf)
            srv.guard_pre = int(cj.get("ofdm_tx_zero_prefix", 128))
            srv.guard_post = int(cj.get("ofdm_tx_zero_postfix", 128))
        except Exception as exc:  # noqa: BLE001 -- markers are cosmetic
            print("[csi] conf parse for guard markers failed: %s" % exc)
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
.csi-cards{display:grid;gap:1rem;padding:1rem;align-items:start;
  grid-template-columns:repeat(auto-fit,minmax(520px,1fr))}
/* No fixed card width. The old 620px card gave 586px of content while the views
   needed 588, so every panel was clipped by 2px: sizing a card by arithmetic that
   has to be redone whenever a panel changes is the bug, not the number. The
   canvases now stretch to whatever the grid gives them. */
.csi-card{min-width:0}
/* Both views need 588px of content and the card gives 606, but a future size change
   should degrade to a scrollbar rather than silently clip a panel off the edge. */
.csi-plots{min-width:0}
/* A stale card dims its plots but NOT its header, so the badge that explains the
   dimming does not dim along with the thing it is explaining. */
.csi-card.stale .csi-plots{opacity:.4}
.csi-plots{display:grid;grid-template-columns:1fr 1fr;gap:.75rem 1rem}
.csi-phase-stack{display:flex;flex-direction:column;gap:.35rem}
.csi-h-half canvas{height:74px}
/* (retired) The quality strip sat in the SAME grid cell as the magnitude panel, directly
   under it, so the two share one subcarrier axis exactly. A strip in its own
   full-width row would be a different pixels-per-subcarrier scale, and a null would
   appear at two different x positions in two panels that describe the same tone. */
.csi-adc .csi-plot{grid-column:1 / -1}
.csi-head{grid-column:1 / -1;margin-top:.5rem}
.csi-head-lbl{font-size:.7rem;color:var(--tblr-secondary);margin-bottom:.15rem}
.csi-head-pct{font-variant-numeric:tabular-nums}
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
.csi-plot canvas{display:block;width:100%;background:var(--tblr-bg-surface-tertiary);
  border:1px solid var(--tblr-border-color);border-radius:4px}
/* Heights live here, widths come from the grid. Bigger than the first pass: at
   120px a 64-subcarrier trace had under 2px per subcarrier. */
.csi-h-line canvas{height:190px}
.csi-h-wf   canvas{height:190px}
.csi-h-cons canvas{height:250px}
.csi-h-adc  canvas{height:220px}
.csi-stage{min-width:0}
.csi-plot{min-width:0}
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
<div id="sync"></div>
<div class="csi-cards" id="ants"></div>
<script>
// Canvas sizes are MEASURED from the layout every time it changes, not declared
// here: the panels stretch with the card, so a hard-coded width could only ever be
// wrong. Heights come from the .csi-h-* classes. See fitCard().
// Full scale for the sample container: the 14-bit ADC is MSB-aligned in int16, so
// the rail really is 32768 and not the converter's 8191 (device/README.md:239,
// Full scale of the int16 sample CONTAINER (the absolute converter mapping
// is unmeasured, DEMO_VERIFICATION.md 2.19). The PARSER is the single
// page-side source: it stamps `full_scale` on every record and drawAdc
// reads the record. The ADC2 wire does not carry it, so a sounder-side
// change still means editing the parser constant (second review 2.6).
const ADC_FS=32767;
const GUARD_PRE=__GUARD_PRE__, GUARD_POST=__GUARD_POST__;
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
     rawph:v('--tblr-yellow'),
     pts:'rgba('+v('--tblr-azure-rgb')+',0.55)'};
}
let themeChanged=false;
function applyTheme(t){
  document.documentElement.setAttribute('data-bs-theme',t);
  try{ localStorage.setItem(THEME_KEY,t); }catch(e){}
  document.getElementById('theme').innerHTML=(t==='dark'?SUN:MOON);
  readTheme();
  // The waterfall's existing rows are in the OLD palette's background, so a theme
  // change is the one repaint that does have to restart it.
  themeChanged=true; redrawAll(); themeChanged=false;
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
// `extra` goes in the title row and must stay small: it is a badge slot. Anything
// with a body of its own goes in `after`, below the x axis, or it becomes a flex item
// inside the caption and squashes the title it was meant to annotate.
function frame(title, cls, ylabels, xlabels, extra, after){
  return '<div class="csi-plot '+cls+'"><div class="csi-plot-title"><span>'+title+'</span>'
    +(extra||'')+'</div><div class="csi-stage">'+yAxis(ylabels)
    +'<canvas></canvas></div>'
    +'<div class="csi-x-axis">'+xlabels.map(t=>'<span>'+t+'</span>').join('')+'</div>'
    +(after||'')+'</div>';
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
  const clip='<span class="badge bg-red-lt text-red csi-clip" hidden>clipping</span>';

  wrap.innerHTML=
    '<div class="card-header py-2">'
     +'<h3 class="card-title">RX antenna '+ant+'</h3>'
     +'<div class="card-actions d-flex gap-1">'
       +'<span class="badge bg-orange-lt text-orange csi-stale" hidden></span>'
     +'</div>'
    +'</div>'
    +'<div class="card-body p-3">'
     +'<ul class="nav nav-underline mb-3 csi-tabs">'
       +'<li class="nav-item"><a href="#" class="nav-link active" data-view="channel">Channel</a></li>'
       +'<li class="nav-item"><a href="#" class="nav-link" data-view="adc">ADC</a></li>'
     +'</ul>'
     +'<div class="csi-plots csi-view" data-view="channel">'
      +frame('|H| (dB) vs subcarrier','csi-h-line',magLabels(),['','',''],off)
      // Raw above corrected [user]: raw = arg(H) exactly as measured (window
      // back-off ramp + per-run offset); corrected = de-ramped and run-anchored.
      +'<div class="csi-phase-stack">'
      +frame('phase (raw, rad)','csi-h-half',['1.0π','0.0π','-1.0π'],['','',''])
      +frame('phase (corrected, rad)','csi-h-half',['1.0π','0.0π','-1.0π'],['','',''])
      +'</div>'
      +frame('waterfall |H| (time down)','csi-h-wf',['older','','now'],['','',''])
      +frame('constellation (equalized U)','csi-h-cons',
             [formatAxisValue(CONS_R),'0.00',formatAxisValue(-CONS_R)],
             [formatAxisValue(-CONS_R),'I','+'+formatAxisValue(CONS_R)])
     +'</div>'
     +'<div class="csi-plots csi-adc csi-view" data-view="adc" hidden>'
      +frame('raw ADC min/max envelope, whole slot','csi-h-adc',
             ['','','','',''],['0','sample','end'],clip)
      // The trace is FITTED to the slot, so the absolute question it cannot answer
      // ("how much converter range am I using") gets its own fixed-scale widget.
      +'<div class="csi-head">'
        +'<div class="d-flex justify-content-between csi-head-lbl">'
          +'<span>converter range used</span><span class="csi-head-pct"></span></div>'
        +'<div class="progress progress-sm"><div class="progress-bar csi-head-bar"'
        +' style="width:0%"></div></div></div>'
     +'</div>'
     +'<div class="text-secondary tnum mt-3 csi-status" style="font-size:.75rem"></div>'
     +'<div class="text-secondary tnum mt-1 csi-adc-status" style="font-size:.75rem"></div>'
    +'</div>';
  document.getElementById('ants').appendChild(wrap);
  const cvs=[...wrap.querySelectorAll('.csi-view canvas')];
  cards[ant]={magCv:cvs[0],rawPhaseCv:cvs[1],phaseCv:cvs[2],wfCv:cvs[3],
              consCv:cvs[4],adcCv:cvs[5],dim:null,wfimg:null,
              status:wrap.querySelector('.csi-status'),
              adcStatus:wrap.querySelector('.csi-adc-status'),
              el:wrap,badge:wrap.querySelector('.csi-stale'),
              off:wrap.querySelector('.csi-off'),
              clip:wrap.querySelector('.csi-clip'),
              adcTitle:wrap.querySelector('.csi-adc .csi-plot-title span'),
              adcY:wrap.querySelectorAll('.csi-adc .csi-y-axis span'),
              headPct:wrap.querySelector('.csi-head-pct'),
              headBar:wrap.querySelector('.csi-head-bar'),
              xax:wrap.querySelectorAll('.csi-view[data-view=channel] .csi-x-axis'),
              lastCsi:-1,lastCns:-1,lastAdc:-1,
              csiRec:null,cnsRec:null,adcRec:null,frame:0};
  // Tabs are per card so you can watch one antenna's ADC while another shows its
  // channel, which is how you find the one converter that is actually clipping.
  // Measure once the card is in the document, and again whenever the grid reflows.
  fitCard(cards[ant]);
  if(cardObserver) cardObserver.observe(wrap);
  wrap.querySelectorAll('.csi-tabs .nav-link').forEach(a=>{
    a.addEventListener('click',e=>{
      e.preventDefault();
      const want=a.dataset.view;
      wrap.querySelectorAll('.csi-tabs .nav-link').forEach(
        b=>b.classList.toggle('active',b===a));
      wrap.querySelectorAll('.csi-view').forEach(
        v=>{v.hidden=(v.dataset.view!==want);});
      // A canvas in a hidden div measures 0, so anything drawn while the tab was
      // closed went nowhere. Re-fit and repaint the moment it becomes visible.
      const card=cards[ant];
      fitCard(card);
      if(card.csiRec) drawCsi(card,card.csiRec,false);
      if(card.cnsRec) drawCons(card,card.cnsRec);
      if(card.adcRec) drawAdc(card,card.adcRec);
    });
  });
}

// Size each backing store to the box CSS gave it, at device resolution so a bigger
// panel is a sharper one rather than the same pixels stretched. The waterfall is the
// exception: it scrolls itself with drawImage and writes rows with putImageData,
// which ignores the transform, so it stays in device pixels with no transform and
// its history is dropped on a resize rather than rescaled into something untrue.
function fitCanvas(cv, useDpr){
  const w=Math.max(1,cv.clientWidth), h=Math.max(1,cv.clientHeight);
  const dpr=useDpr ? (window.devicePixelRatio||1) : 1;
  const bw=Math.max(1,Math.round(w*dpr)), bh=Math.max(1,Math.round(h*dpr));
  const ctx=cv.getContext('2d');
  // Assigning canvas.width WIPES the bitmap even when the value is unchanged, so
  // the guard is not an optimisation: without it every observer callback erases
  // the waterfall's history. `changed` is reported so callers can tell a real
  // resize from a no-op.
  const changed = (cv.width!==bw || cv.height!==bh);
  if(changed){ cv.width=bw; cv.height=bh; }
  ctx.setTransform(dpr,0,0,dpr,0,0);
  return {w:w,h:h,ctx:ctx,bw:bw,bh:bh,changed:changed};
}
function fitCard(card, force){
  const d={};
  d.mag  = fitCanvas(card.magCv,  true);
  d.rawph= fitCanvas(card.rawPhaseCv,true);
  d.phase= fitCanvas(card.phaseCv,true);
  d.cons = fitCanvas(card.consCv, true);
  d.adc  = fitCanvas(card.adcCv,  true);
  d.wf   = fitCanvas(card.wfCv,   false);
  card.dim=d;
  card.mag=d.mag.ctx; card.rawph=d.rawph.ctx; card.phase=d.phase.ctx;
  card.cons=d.cons.ctx; card.adc=d.adc.ctx; card.wf=d.wf.ctx;
  // Only when the waterfall's device size ACTUALLY changed: its history lives in
  // the bitmap and cannot be resampled honestly, so a real resize has to restart
  // it -- but a no-op refit must not. The card's height changes whenever the
  // status line's text changes width, which is every frame, so an unconditional
  // clear here restarts the waterfall continuously.
  if(d.wf.changed || !card.wfimg || force){
    card.wfimg=card.wf.createImageData(d.wf.bw,1);
    card.wf.fillStyle=C.bg; card.wf.fillRect(0,0,d.wf.bw,d.wf.bh);
  }
}

// Grid only: the labels are HTML now. Horizontal quarters plus the DC centre line.
function grid(ctx,w,h){
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle=C.grid; ctx.lineWidth=1;
  for(let i=0;i<=4;i++){const y=(h*i/4)|0;
    ctx.beginPath();ctx.moveTo(0,y+.5);ctx.lineTo(w,y+.5);ctx.stroke();}
  const xz=(w/2)|0; ctx.beginPath();ctx.moveTo(xz+.5,0);ctx.lineTo(xz+.5,h);ctx.stroke();
}

function line(ctx,vals,ymin,ymax,color,w,h){
  const n=vals.length; ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.beginPath();
  let started=false;
  for(let k=0;k<n;k++){
    const v=vals[k];
    if(v===null){started=false;continue;}
    const x=w*k/(n-1);
    const y=h-(v-ymin)/(ymax-ymin)*h;
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
  // Gutters in document order: mag, raw phase, corrected phase, waterfall.
  // The constellation (index 4) keeps its own I/Q labels (Opus review M14:
  // adding the phase stack shifted these indices and the waterfall lost its
  // labels).
  for(let i=0;i<4;i++){
    const sp=card.xax[i].querySelectorAll('span');
    for(let j=0;j<3;j++) sp[j].textContent=lab[j];
  }
}

function drawCsi(card,c,advance){
  card.frame=c.frame; card.csiRec=c;
  setScAxis(card,c.sc);
  // Magnitude: FIXED axis, never re-ranged. Set with --mag-top / --mag-span.
  const top=MAG_TOP, bot=MAG_BOT;
  const dm=card.dim.mag, dp=card.dim.phase, dw=card.dim.wf;
  grid(card.mag,dm.w,dm.h);
  line(card.mag,c.mag_db,bot,top,C.mag,dm.w,dm.h);
  // A fixed axis can hide the trace entirely if the level moves off scale, so say
  // so rather than showing an innocent-looking empty panel.
  const fin=c.mag_db.filter(v=>v!==null);
  card.off.hidden=!(fin.length&&(Math.max(...fin)>top||Math.min(...fin)<bot));
  // Both phase panels: fixed -pi..+pi. Raw = as measured; corrected =
  // de-ramped + run-anchored (the sounder does both transforms).
  const dr=card.dim.rawph;
  grid(card.rawph,dr.w,dr.h);
  if(c.raw_ph) line(card.rawph,c.raw_ph,-Math.PI,Math.PI,C.rawph,dr.w,dr.h);
  grid(card.phase,dp.w,dp.h);
  line(card.phase,c.phase,-Math.PI,Math.PI,C.phase,dp.w,dp.h);
  // waterfall: scroll up 1px, draw new bottom row coloured by magnitude
  if(advance!==false){
    card.wf.drawImage(card.wf.canvas,0,-1);
    const n=c.mag_db.length, d=card.wfimg.data, WW=dw.bw;
    for(let x=0;x<WW;x++){
      const k=Math.min(n-1,(x*n/WW)|0), v=c.mag_db[k];
      let col=[0,0,0];
      if(v!==null) col=jet((v-bot)/(top-bot));
      d[4*x]=col[0];d[4*x+1]=col[1];d[4*x+2]=col[2];d[4*x+3]=255;
    }
    card.wf.putImageData(card.wfimg,0,dw.bh-1);
  }
  card.status.textContent='frame '+c.frame+' · '+c.sc+' subcarriers · '
     +(c.rate/1e6).toFixed(2)+' MS/s · peak '+formatScaled(c.peak_db,'db');
}

function drawAdc(card,a){
  card.adcRec=a;
  const ctx=card.adc, d=card.dim.adc, AW=d.w, AH=d.h;
  const FS=a.full_scale||ADC_FS;
  ctx.clearRect(0,0,AW,AH);
  // Power envelope in dBFS on a FIXED 0..-80 axis [user 2026-08-30: the raw
  // I/Q min/max bands read as a noise block -- "pretty messy"]. One line, the
  // burst structure visible: where energy starts and ends against the nominal
  // guard seats (dashed), which is the live landing view. Clip catching is
  // unchanged: the amplitude per column is still the max over EVERY sample it
  // covers, so one clipped sample pins its column at 0 dBFS.
  const n=a.cols, DB_BOT=-80;
  const amp=new Array(n);
  for(let k=0;k<n;k++)
    amp[k]=Math.max(Math.abs(a.i_min[k]),Math.abs(a.i_max[k]),
                    Math.abs(a.q_min[k]),Math.abs(a.q_max[k]));
  ctx.strokeStyle=C.grid; ctx.lineWidth=1;
  for(let i=0;i<=4;i++){const y=(AH*i/4)|0;
    ctx.beginPath();ctx.moveTo(0,y+.5);ctx.lineTo(AW,y+.5);ctx.stroke();}
  // Nominal guard seats: signal should occupy [128, samps-128) of the slot.
  if(a.samps>GUARD_PRE+GUARD_POST){
    ctx.strokeStyle=C.warn; ctx.setLineDash([3,3]);
    for(const fx of [GUARD_PRE/a.samps, (a.samps-GUARD_POST)/a.samps]){
      const x=(AW*fx)|0;
      ctx.beginPath();ctx.moveTo(x+.5,0);ctx.lineTo(x+.5,AH);ctx.stroke();
    }
    ctx.setLineDash([]);
  }
  ctx.strokeStyle=C.mag; ctx.lineWidth=1.5; ctx.beginPath();
  let started=false;
  for(let k=0;k<n;k++){
    const db=20*Math.log10(Math.max(amp[k],1)/FS);
    const y=Math.min(AH-1,(db/DB_BOT)*AH);
    const x=AW*k/(n-1);
    if(!started){ctx.moveTo(x,y);started=true;}else ctx.lineTo(x,y);
  }
  ctx.stroke();
  for(let i=0;i<=4;i++)
    if(card.adcY[i]) card.adcY[i].textContent=(DB_BOT*i/4).toFixed(0);
  const pct=100*a.peak/FS;
  card.adcTitle.textContent='pilot power envelope (dBFS), slot '+(a.slot>=0?a.slot:'?')+', nominal guards dashed';
  // The fixed-scale half of the panel. Under-driving is the failure we actually have,
  // so it gets a colour of its own rather than sharing "fine" with a healthy level.
  card.headBar.style.width=Math.max(0.5,Math.min(100,pct)).toFixed(2)+'%';
  card.headBar.className='progress-bar csi-head-bar '+
    (a.clipped>0||pct>=95 ? 'bg-danger' : pct<10 ? 'bg-warning' : 'bg-success');
  card.headPct.textContent=pct.toFixed(1)+'% of full scale'
    +(pct<10 ? ' (under-driven)' : '');
  // Clipping anywhere in the frame lights the badge, not just on the drawn slot.
  const anyClip=(a.any_clipped===undefined?a.clipped:a.any_clipped);
  card.clip.hidden=(anyClip===0);
  const anyPct=100*(a.any_peak===undefined?a.peak:a.any_peak)/FS;
  card.adcStatus.textContent='frame '+a.frame+' \u00b7 '+a.samps+' samples \u00b7 pilot peak '
    +a.peak+' of '+FS+' ('+pct.toFixed(1)+'% FS) \u00b7 all slots: peak '
    +anyPct.toFixed(1)+'% FS, '
    +(anyClip?anyClip+' sample(s) clipped':'no clipping');
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
  const ctx=card.cons, d=card.dim.cons;
  ctx.clearRect(0,0,d.w,d.h);
  // I and Q must share a scale or the cloud shears, so the plot is a centred square
  // however wide the card gets.
  const S=Math.min(d.w,d.h), cx=d.w/2, cy=d.h/2, R=S/2/CONS_R;
  ctx.strokeStyle=C.grid; ctx.lineWidth=1; ctx.beginPath();
  ctx.moveTo(cx,cy-S/2);ctx.lineTo(cx,cy+S/2);
  ctx.moveTo(cx-S/2,cy);ctx.lineTo(cx+S/2,cy);ctx.stroke();
  ctx.strokeRect(cx-S/2+.5,cy-S/2+.5,S-1,S-1);
  // Both marks are CENTRED on their value. fillRect places its top-left corner at the
  // coordinate given, so drawing the received dots without the half-size back-off put
  // every one of them down and right of the ideal marker it belongs to, by half a dot.
  // Small, systematic, and in one direction: exactly the kind of offset that reads as
  // the data not landing where it should.
  const DOT=2.0, REF=6.0;
  ctx.fillStyle=C.pts;                     // received points
  for(const p of cn.pts){ctx.fillRect(cx+p[0]*R-DOT/2,cy-p[1]*R-DOT/2,DOT,DOT);}
  ctx.fillStyle=C.warn;                    // ideal alphabet, drawn as an open ring so
  ctx.lineWidth=1.5;                       // it cannot bury the cloud it is marking
  ctx.strokeStyle=C.warn;
  for(const p of idealPts(cn.mod)){
    ctx.beginPath();
    ctx.arc(cx+p[0]*R, cy-p[1]*R, REF/2, 0, 2*Math.PI);
    ctx.stroke();
  }
}

// A theme change has to repaint every canvas from the last record, because a
// stalled stream will not repaint them for us. The waterfall history lives in the
// bitmap and cannot be recoloured, so it is cleared to the new background rather
// than left as a rectangle of the old theme.
function redrawAll(){
  for(const t in syncCards){
    if(syncCards[t].rec) drawSyncCard(t, syncCards[t].rec);
  }
  for(const a in cards){
    const card=cards[a];
    fitCard(card, themeChanged);       // clears the waterfall only when it must
    if(card.csiRec) drawCsi(card,card.csiRec,false);
    if(card.cnsRec) drawCons(card,card.cnsRec);
    if(card.adcRec) drawAdc(card,card.adcRec);
  }
}

// One observer for the whole strip. Resizes arrive in bursts while a window is
// dragged, so coalesce to the next frame rather than re-fitting per event.
let resizePending=false;
const cardObserver=(typeof ResizeObserver!=='undefined') ? new ResizeObserver(()=>{
  if(resizePending) return;
  resizePending=true;
  requestAnimationFrame(()=>{ resizePending=false; redrawAll(); });
}) : null;

let pktCount=0,t0=Date.now();
// ---- beacon sync / CFO panel (AP-32) --------------------------------------
// One card per client tid, matching how every other stream here is keyed.
const SYNC_SHOW=120, SYNC_QUIET_MS=2500, SYNC_DEAD_MS=60000;
// The lane MEASURED a 1.9 kHz run-to-run spread with the clocks locked (two
// zero-injection runs, +756 vs -1147 Hz), because a short correlation lag turns
// a tiny phase error into a large apparent frequency. Anything under this is an
// instrument reading, not a carrier offset -- BACKLOG AP-30.
const CFO_NOISE_HZ=2000;
const syncCards={};

function makeSyncCard(tid){
  const wrap=document.createElement('div');
  wrap.className='card csi-card';
  wrap.innerHTML='<div class="card-body">'
    +'<div class="d-flex align-items-center justify-content-between mb-2">'
      +'<h3 class="card-title mb-0">beacon sync'+(tid!=='0'?(' [UE '+tid+']'):'')+'</h3>'
      +'<span class="badge bg-secondary-lt sync-chip">--</span></div>'
    +frame('resid vs the anchored grid (samples)','csi-h-line',
           ['','','0','',''],['older','frame','now'])
    +'<div class="text-secondary tnum mt-2 sync-read" style="font-size:.75rem"></div>'
    +'</div>';
  document.getElementById('sync').appendChild(wrap);
  syncCards[tid]={cv:wrap.querySelector('canvas'),
                  chip:wrap.querySelector('.sync-chip'),
                  read:wrap.querySelector('.sync-read'),
                  plot:wrap.querySelector('.csi-plot'),
                  ylab:[...wrap.querySelectorAll('.csi-y-axis span')],
                  rec:null};
}

// A segment is a stretch over which resid is comparable. It ends ONLY at a
// re-anchor (state 3), after which resid is measured against a REPLACED
// reference, and at a frame counter that goes backwards (the launcher's retry
// loop restarts the sounder). State 5 does NOT end it: that branch keeps the
// previous anchor, so resid stays comparable straight across it -- truncating
// there would blank the trace exactly when a failed re-anchor makes it most
// diagnostic.
function lastSyncSegment(hist){
  let start=0;
  for(let i=1;i<hist.length;i++){
    if(hist[i].frame < hist[i-1].frame) start=i;
    else if(hist[i-1].state===3) start=i;
  }
  return hist.slice(start);
}

function syncChip(last, age){
  if(!last || age>=SYNC_DEAD_MS) return ['NOT SYNCED','bg-red-lt'];
  switch(last.state){
    case 1: return ['LOCKED','bg-green-lt'];
    case 2: return ['HOLD PENDING','bg-yellow-lt'];
    case 3: return ['RE-ANCHORED','bg-orange-lt'];
    case 4: return ['WEAK BEACON','bg-yellow-lt'];
    case 5: return ['RE-ANCHOR FAILED','bg-red-lt'];
    // An unknown code means the page is older than the sounder. Say so rather
    // than defaulting to green, which would assert health we cannot vouch for.
    default: return ['STATE '+last.state+'?','bg-secondary-lt'];
  }
}

function drawSyncCard(tid, sync){
  if(!syncCards[tid]) makeSyncCard(tid);
  const card=syncCards[tid];
  card.rec=sync;
  const hist=(sync.hist||[]).slice(-SYNC_SHOW);
  const age=(sync.age_ms===null||sync.age_ms===undefined)?Infinity:sync.age_ms;
  const last=hist.length?hist[hist.length-1]:null;
  let [label,cls]=syncChip(last,age);
  if(last && age>=SYNC_QUIET_MS && age<SYNC_DEAD_MS){
    label+=' \u00b7 quiet '+(age/1000).toFixed(1)+'s';
  }
  card.chip.textContent=label;
  card.chip.className='badge '+cls;
  if(card.plot) card.plot.style.opacity=(age>=SYNC_QUIET_MS)?'0.4':'1';

  const d=fitCanvas(card.cv,true), ctx=d.ctx;
  ctx.clearRect(0,0,d.w,d.h);
  if(!hist.length){card.read.textContent='no detections yet';return;}

  const seg=lastSyncSegment(hist);
  // tol is a uint32 and 0 is a legal, meaningful value ("reject anything
  // off-grid"), so test for presence rather than truthiness.
  const TOL=(last && last.tol!==undefined && last.tol!==null)?last.tol:1024;
  // The y range FOLLOWS the tolerance. Hardcoding it meant a retune pushed the
  // limit lines off-canvas while the axis labels kept claiming the old span.
  const YR=Math.max(64, Math.round(TOL*1.3));
  if(card.ylab.length===5){
    const v=[YR, YR/2, 0, -YR/2, -YR];
    card.ylab.forEach((el,i)=>{el.textContent=(v[i]>0?'+':'')+Math.round(v[i]);});
  }
  const f0=seg[0].frame, f1=seg[seg.length-1].frame;
  const span=Math.max(1,f1-f0);
  const X=f=>((f-f0)/span)*(d.w-1);
  // Clamp so an off-scale HOLD is pinned to the edge instead of drawn off the
  // canvas, which silently hid the beacon-moved events the panel exists to show.
  const Y=v=>Math.max(2,Math.min(d.h-2, d.h/2-(v/YR)*(d.h/2-2)));

  ctx.fillStyle='rgba(128,128,128,0.10)';
  ctx.fillRect(0,Y(TOL),d.w,Y(-TOL)-Y(TOL));
  ctx.strokeStyle=C.warn;ctx.lineWidth=1;ctx.setLineDash([3,3]);
  ctx.beginPath();
  ctx.moveTo(0,Y(TOL));ctx.lineTo(d.w,Y(TOL));
  ctx.moveTo(0,Y(-TOL));ctx.lineTo(d.w,Y(-TOL));ctx.stroke();
  ctx.setLineDash([]);
  ctx.strokeStyle=C.grid;ctx.beginPath();
  ctx.moveTo(0,Y(0));ctx.lineTo(d.w,Y(0));ctx.stroke();

  // Re-anchors as vertical rules, under the data.
  ctx.strokeStyle=C.warn;ctx.lineWidth=1.5;
  for(const p of seg){ if(p.state===3){
    ctx.beginPath();ctx.moveTo(X(p.frame),0);ctx.lineTo(X(p.frame),d.h);ctx.stroke(); } }
  for(const p of seg){
    // States 3/4/5 carry no measured resid -- 4 in particular is emitted with a
    // hardcoded 0 because the SNR gate runs BEFORE any residual is computed, so
    // plotting it would paint a fabricated dot dead centre of the accept band:
    // the exact picture of health the walkthrough teaches operators to trust.
    if(p.state!==1 && p.state!==2) continue;
    ctx.fillStyle=(p.state===1)?C.mag:C.warn;
    ctx.fillRect(X(p.frame)-1.5,Y(p.resid)-1.5,3,3);
  }
  // WEAK gets a rug mark along the bottom instead: visible, but never mistaken
  // for a residual measurement.
  ctx.fillStyle=C.warn;
  for(const p of seg){ if(p.state===4) ctx.fillRect(X(p.frame)-1,d.h-4,2,4); }

  // Fit LOCKED only. A HOLD is by construction |resid| > tol -- the scatter the
  // hold-off logic exists to reject -- so including it let one outlier move the
  // slope ~100x past the precision this figure is quoted at.
  const loc=seg.filter(p=>p.state===1);
  const esc=hist.filter(p=>p.state===3);
  const tail=esc.length?esc[esc.length-1]:null;
  const note=(tail?('  |  last re-anchor '+tail.shift+' samp'):'')
            +(seg.some(p=>p.state===4)?'  |  weak beacon seen':'');
  if(loc.length<3){
    card.read.textContent=loc.length+' locked detection(s) in this segment'+note;
    return;
  }
  let sx=0,sy=0; for(const p of loc){sx+=p.frame;sy+=p.resid;}
  const n=loc.length, mx=sx/n, my=sy/n;
  let num=0,den=0;
  for(const p of loc){num+=(p.frame-mx)*(p.resid-my);den+=(p.frame-mx)*(p.frame-mx);}
  const sfr=loc[n-1].sfr||1, fc=loc[n-1].fc||0;
  const tppm=(den>0?num/den:0)/sfr*1e6;
  let cs=0; for(const p of loc)cs+=p.cfo;
  const cm=cs/n;
  let cq=0; for(const p of loc)cq+=(p.cfo-cm)*(p.cfo-cm);
  // SEM, not the population SD: the figure quoted is the MEAN, so its error bar
  // shrinks as sqrt(n). Printing the per-sample spread made a resolved offset
  // look unresolvable.
  const sem=Math.sqrt(cq/n)/Math.sqrt(n);
  const cppm=(fc>0?cm/fc*1e6:0), cppmsem=(fc>0?sem/fc*1e6:0);
  const noisy=Math.abs(cm)<CFO_NOISE_HZ;
  card.read.textContent=
    'timing '+tppm.toFixed(4)+' ppm (resid slope)  |  carrier '
    +cppm.toFixed(3)+' +/- '+cppmsem.toFixed(3)+' ppm ('+cm.toFixed(0)+' +/- '
    +sem.toFixed(0)+' Hz'+(noisy?', inside the ~2 kHz phase-noise floor':'')
    +')  |  '+n+' locked over '+span+' frames'+note;
}

function drawSync(sync){
  const tids=Object.keys(sync);
  // With no SYN1 at all the server sends {} and there is no tid to iterate, so
  // without this the card never appears and the documented NOT SYNCED state is
  // unreachable -- a link that never locked would read as a broken GUI.
  if(!tids.length){
    if(!Object.keys(syncCards).length) drawSyncCard('0',{hist:[],age_ms:null});
    return;
  }
  for(const tid of tids) drawSyncCard(tid, sync[tid]);
}

function onData(obj){
  if(obj.sync) drawSync(obj.sync);
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
    if(rec.adc && rec.adc.frame!==card.lastAdc){
      drawAdc(card,rec.adc); card.lastAdc=rec.adc.frame;
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
