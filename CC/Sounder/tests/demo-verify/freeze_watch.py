#!/usr/bin/env python3
"""Watch a demo run's sounder log for the HS-227 TX playout freeze. Stdlib only;
runs ON the rig host and reads the local log, so it adds no network traffic.

The rule: any channel's `played` (from the TX_BANK_STATUS on each `TX status:`
line) is unchanged over 3 consecutive such lines, or the device's `PLAYOUT FROZEN`
ERROR reaches the log. A healthy run prints `TX status:` only on problem events and
its `played` grows between them; a frozen channel prints one every health period
with `played` stuck. The HS-227 race strands one channel at a time, so every
channel is watched, not only ch0 (in D8I and D8I2 both froze together). Validated
on D8I (trips at 781 s) and D8I2 (1994 s), silent on D8, D8I3 and both S0 runs
(DEMO_VERIFICATION 9.41-9.46).

usage:
  freeze_watch.py --run-dir <SOUNDER_DIR>/ap79_runs --tag <TAG>   # live, beside fstage_run.sh
  freeze_watch.py --log <log> --pid <sounder pid>                  # live, explicit
  freeze_watch.py --replay <log>                                   # offline, a finished log
Live mode polls every 10 s, prints one line on a trip and exits 0; it also exits 0
when the sounder exits. It holds the log open, so fstage_run.sh filing the log
into the stage directory after the run does not cut off its last lines. Exit 2:
the run never started (--run-dir waits 90 s for run_rung.sh's pid and record).
--replay exits 1 when the log froze, 0 when it did not."""
import argparse, os, re, sys, time

PLAYED = re.compile(r"(ch\d+):[^ ;]*?played=(\d+)")
DEV_NS = re.compile(r" at (\d+) ns")


def utc():
    return time.strftime("%H:%M:%S", time.gmtime())


class Rule:
    def __init__(self):
        self.last = {}

    def feed(self, line):
        """Return a trip message for this line, or None."""
        if "PLAYOUT FROZEN" in line:
            return "FREEZE (device detector): %s" % line[:420]
        if "TX status:" not in line:
            return None
        stuck = []
        for ch, played in PLAYED.findall(line):
            seen = self.last[ch] = (self.last.get(ch, []) + [played])[-3:]
            if len(seen) == 3 and len(set(seen)) == 1:
                stuck.append(ch)
        if not stuck:
            return None
        t = DEV_NS.search(line)
        return "FREEZE detected on %s, dev %.1f s: %s" % (
            ",".join(stuck), int(t.group(1)) / 1e9 if t else -1, line[:420])


def replay(path):
    rule = Rule()
    with open(path, errors="replace") as f:
        for n, line in enumerate(f, 1):
            msg = rule.feed(line.rstrip("\n"))
            if msg:
                print("line %d: %s" % (n, msg))
                return 1
    print("no freeze in %s" % path)
    return 0


def wait_for_run(run_dir, tag, timeout_s=90):
    pid_f, cur_f = os.path.join(run_dir, tag + ".pid"), os.path.join(run_dir, tag + ".current")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            pid = int(open(pid_f).read().strip())
            cur = open(cur_f).read().strip()
            if open("/proc/%d/comm" % pid).read().strip() == "sounder" and cur:
                return os.path.join(run_dir, cur + ".log"), pid
        except (OSError, ValueError):
            pass
        time.sleep(2)
    return None, None


def watch(path, pid):
    print("watching %s sounder %d from %s UTC" % (path, pid, utc()), flush=True)
    rule, f, buf = Rule(), None, ""
    while True:
        alive = os.path.exists("/proc/%d" % pid)
        if f is None:
            try:
                f = open(path, errors="replace")
            except FileNotFoundError:
                pass
        buf += f.read() if f else ""
        lines = buf.split("\n")
        buf = lines.pop() if alive else ""  # a dead writer's last line is whole
        for line in lines:
            msg = rule.feed(line)
            if msg:
                print("%s UTC %s" % (utc(), msg), flush=True)
                return 0
        if not alive:
            print("run ended %s UTC, no freeze" % utc(), flush=True)
            return 0
        time.sleep(10)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--replay")
    ap.add_argument("--run-dir")
    ap.add_argument("--tag")
    ap.add_argument("--log")
    ap.add_argument("--pid", type=int)
    a = ap.parse_args()
    if a.replay:
        return replay(a.replay)
    if a.run_dir and a.tag:
        path, pid = wait_for_run(a.run_dir, a.tag)
        if not path:
            print("no running sounder for %s in %s" % (a.tag, a.run_dir), flush=True)
            return 2
    elif a.log and a.pid:
        path, pid = a.log, a.pid
    else:
        ap.error("give --replay, or --run-dir and --tag, or --log and --pid")
    return watch(path, pid)


if __name__ == "__main__":
    sys.exit(main())
