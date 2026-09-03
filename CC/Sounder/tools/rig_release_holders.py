#!/usr/bin/env python3
"""Kill every local process that can be holding a board's RX stream.

WHY THIS EXISTS AND WHY IT SCANS /proc. `csi_server.py --launch` spawns a
`bash -lc` retry loop as a child which spawns the sounder as a grandchild, so
killing the server orphans the loop; it restarts a sounder, that sounder holds
both boards' RX streams, and every later run is refused at setSampleRate(RX)
with an EMPTY log. Six consecutive gate runs died that way unnoticed.

pkill/pgrep -f cannot be used here: the pattern appears in the invoking command
line, so it matches and kills the caller. That has already happened twice today
(an ssh session and a gate script). Scanning /proc and excluding our own
ancestry is the only form that cannot do it.

The teardown helper's advice for a held board is `sudo systemctl restart
SoapySDRServer`, which needs a password this bench does not grant. It is also
heavier than necessary: the holder is a LOCAL process, and killing it releases
the stream, which is what this does.
"""
import os, signal, sys, time

MARKERS = ("build/sounder", "csi_gui/csi_server.py", "for a in 1 2 3 4")


def ancestry(pid):
    out, seen = set(), 0
    while pid > 1 and seen < 40:
        out.add(pid)
        seen += 1
        try:
            with open("/proc/%d/stat" % pid) as f:
                pid = int(f.read().split(") ", 1)[1].split()[1])
        except Exception:
            break
    return out


def main():
    mine = ancestry(os.getpid())
    victims = []
    for e in os.listdir("/proc"):
        if not e.isdigit():
            continue
        pid = int(e)
        if pid in mine:
            continue
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except Exception:
            continue
        if any(m in cmd for m in MARKERS):
            victims.append((pid, cmd[:90]))
    for pid, cmd in victims:
        print("kill %d  %s" % (pid, cmd))
        try:
            os.kill(pid, signal.SIGINT)
        except Exception:
            pass
    if victims:
        time.sleep(4)
        for pid, _ in victims:
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
        time.sleep(2)
    print("cleaned %d process(es)" % len(victims))
    return 0


if __name__ == "__main__":
    sys.exit(main())
