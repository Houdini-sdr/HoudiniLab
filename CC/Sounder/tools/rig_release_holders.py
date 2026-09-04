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

# The third marker was "for a in 1 2 3 4", copied from csi_server.py's retry
# loop -- change that loop's bounds and this silently stops matching and leaves
# the orphan alive. Match the teardown call the loop always makes instead.
MARKERS = ("build/sounder", "csi_gui/csi_server.py", "csi_gui/teardown_framer.py")


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
    def still_ours(pid, cmd):
        """Is `pid` STILL the process we decided to kill?

        A pid captured before a 4-second wait can be recycled, and this tool
        exists because pattern-matching kills have already hit the wrong process
        twice today. Re-read the cmdline before escalating to SIGKILL.
        """
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as f:
                now = f.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except Exception:
            return False
        return now.startswith(cmd[:40])

    killed = []
    for pid, cmd in victims:
        try:
            os.kill(pid, signal.SIGINT)
        except Exception as exc:  # noqa: BLE001 -- report, never claim success
            print("could not signal %d (%s): %s" % (pid, cmd[:40], exc))
            continue
        killed.append((pid, cmd))
        print("SIGINT %d  %s" % (pid, cmd))
    if killed:
        time.sleep(4)
        for pid, cmd in killed:
            if not still_ours(pid, cmd):
                continue          # exited already, or the pid was recycled
            try:
                os.kill(pid, signal.SIGKILL)
                print("SIGKILL %d" % pid)
            except Exception:
                pass
        time.sleep(2)
    print("cleaned %d process(es)" % len(killed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
