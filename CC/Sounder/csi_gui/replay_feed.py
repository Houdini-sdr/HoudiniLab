#!/usr/bin/env python3
"""Replay a dashboard recording into a running ``csi_server.py``: the canned-data
fallback. The datagrams go out exactly as the sounder sent them, at their
recorded pace, so every panel (channel, constellation, CIR, ADC, beacon sync)
shows real rig data with no radios.

Record during a good run by starting the dashboard with ``--record FILE`` (or
``HOUDINI_CSI_RECORD=FILE`` in its environment, which a scripted run inherits).
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


def schedule(times, speed):
    """Send offsets (s, from the first datagram) for the recorded arrival times."""
    if not times:
        return []
    t0 = times[0]
    return [(t - t0) / speed for t in times]


def load(path, start, duration):
    recs = [(t, d) for t, d in read_recording(path)]
    if not recs:
        return []
    t0 = recs[0][0]
    lo = t0 + start
    hi = t0 + start + duration if duration else float("inf")
    return [(t, d) for t, d in recs if lo <= t < hi]


def play(sock, dest, recs, speed):
    offs = schedule([t for t, _ in recs], speed)
    base = time.monotonic()
    for off, (_, data) in zip(offs, recs):
        wait = base + off - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        sock.sendto(data, dest)


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
    recs = load(a.recording, a.start, a.duration)
    if not recs:
        print("nothing to play in %s (window %.1f s + %.1f s)" % (a.recording, a.start, a.duration))
        return 1
    span = recs[-1][0] - recs[0][0]
    print("replaying %d datagrams over %.1f s to %s:%d%s" % (len(recs), span / a.speed, a.host, a.port,
                                                          ", looping" if a.loop else ""), flush=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        while True:
            play(sock, (a.host, a.port), recs, a.speed)
            if not a.loop:
                return 0
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
