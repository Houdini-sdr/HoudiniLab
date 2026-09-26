#!/usr/bin/env python3
"""Stop every local process that can be holding a board's RX stream.

A sounder left running (started from another window or a harness, or by a
launcher killed without its cleanup) holds both boards' RX streams, and every
later run is refused at setSampleRate(RX) with an EMPTY log. This stops EVERY
sounder, dashboard and teardown on the host, a running --control dashboard
included: SIGINT first, the sounder's own stop (its end-of-run checks, the
clock released), and SIGKILL only for what is still there after
GRACE_S, as DEMO_BENCH_RUNBOOK.md A8 does by hand.

pkill/pgrep -f cannot be used here: the pattern appears in the invoking command
line, so it matches and kills the caller (an ssh session and a gate script
have both been killed that way). Scanning /proc and excluding our own ancestry
is the only form that cannot do it.

Restarting SoapySDRServer also frees a board but needs sudo; the holder is a
LOCAL process, and stopping it is enough.
"""
import os, signal, sys, time

# The programs, never text copied from a script's body: a marker lifted from a
# loop stops matching when the loop changes and leaves the holder alive.
MARKERS = ("build/sounder", "csi_gui/csi_server.py", "csi_gui/teardown_framer.py")
# A clean stop takes about 1 s; a dashboard gives its sounder 10 s
# (csi_server.py STOP_GRACE_S) before its own SIGKILL.
GRACE_S = 10.0


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

        A pid captured before the grace can be recycled, and pattern-matching
        kills have hit the wrong process before. Re-read the cmdline before
        escalating to SIGKILL.
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
        deadline = time.time() + GRACE_S
        while time.time() < deadline and any(still_ours(pid, cmd) for pid, cmd in killed):
            time.sleep(0.5)
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
