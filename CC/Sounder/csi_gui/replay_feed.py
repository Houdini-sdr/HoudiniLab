#!/usr/bin/env python3
"""Replay a dashboard recording into a running ``csi_server.py``: the canned-data
fallback. The datagrams go out exactly as the sounder sent them, at their
recorded pace, so every panel (channel, constellation, CIR, ADC, beacon sync)
shows real rig data with no radios.

Record during a good run by starting the dashboard with ``--record FILE`` (a
scripted run: ``demo_run.sh --record FILE``).
Replay:

    python3 csi_server.py &                      # terminal 1
    python3 replay_feed.py FILE --loop           # terminal 2

``--start`` and ``--duration`` pick a window of the recording (seconds from its
first datagram); ``--speed`` scales the pace. Each loop restarts the frame
counters, which the page treats as a restarted run (the sync card starts a new
segment)."""
import argparse
import socket
import sys
import time

from csi_record import read_recording


def window(path, start, duration):
    """Yield the datagrams in [start, start + duration) s from the recording's
    first one (duration 0: to the end), reading as a stream that stops past the
    window: a recording is up to 2 GB, and a laptop replaying a window of it
    should not need the whole file in memory."""
    lo = hi = None
    for t, d in read_recording(path):
        if lo is None:
            lo = t + start
            hi = t + start + duration if duration else float("inf")
        if t >= hi:  # arrival order: nothing later is in the window
            return
        if t >= lo:
            yield t, d


def load(path, start, duration):
    return list(window(path, start, duration))


def play(sock, dest, recs, speed):
    """Send `recs` (any iterable of (t, datagram)) at their recorded pace; the
    number sent."""
    t0 = base = None
    n = 0
    for t, data in recs:
        if t0 is None:
            t0, base = t, time.monotonic()
        wait = base + (t - t0) / speed - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        sock.sendto(data, dest)
        n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("recording")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9999, help="the dashboard's --udp-port")
    ap.add_argument("--start", type=float, default=0.0, help="seconds into the recording")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to play (0: to the end)")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--loop", action="store_true")
    a = ap.parse_args()
    if a.speed <= 0:
        ap.error("--speed must be positive")
    print("replaying %s (from %.1f s, %s) to %s:%d%s" % (
        a.recording, a.start, "%.1f s" % a.duration if a.duration else "to the end", a.host, a.port,
        ", looping" if a.loop else ""), flush=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        while True:  # each loop reads the window from the file again
            n = play(sock, (a.host, a.port), window(a.recording, a.start, a.duration), a.speed)
            if n == 0:
                print("nothing to play in %s (window %.1f s + %.1f s)" % (a.recording, a.start, a.duration))
                return 1
            print("played %d datagrams" % n, flush=True)
            if not a.loop:
                return 0
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
