# Known answers for freeze_watch.py's rule. Stdlib only. Each check names the
# mutation of the rule that makes it fail.
import importlib.util, os, sys
spec = importlib.util.spec_from_file_location("fw", os.path.join(os.path.dirname(os.path.abspath(__file__)), "freeze_watch.py"))
fw = importlib.util.module_from_spec(spec); spec.loader.exec_module(fw)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)

def status(t_s, ch0, ch1):
    """A TX status line shaped like D8I's (sounder log, the UE's two TX channels)."""
    return ("62:657095 WARNG: TX status: 3 problem event(s), latest TIME_ERROR (code -6) on tx_stream[0] "
            "at %d ns. TX_BANK_STATUS=ch0:acked=1,late=5,under=2,%s;ch1:acked=1,late=4,under=1,%s" % (
                int(t_s * 1e9), "played=%d,clear_drops=0" % ch0 if ch0 is not None else "clear_drops=0",
                "played=%d,clear_drops=0" % ch1))

def run(lines):
    rule = fw.Rule()
    for n, line in enumerate(lines):
        msg = rule.feed(line)
        if msg:
            return n, msg
    return None, None

n, _ = run([status(10, 100, 100), status(15, 200, 200), status(20, 300, 300), status(25, 400, 400)])
check(n is None, "played growing on both channels: no trip (fails if any 3 status lines trip)")

n, msg = run([status(10, 500, 700), status(15, 500, 800), status(20, 500, 900)])
check(n == 2 and "on ch0," in msg + ",", "ch0 stuck over 3 lines: trips on the third (fails if the channel is dropped from the window)")
check(msg is not None and "dev 20.0 s" in msg, "the trip reports the device time of its line (fails if DEV_NS stops matching)")

n, _ = run([status(10, 400, 1), status(15, 500, 2), status(20, 500, 3), status(25, 600, 4)])
check(n is None, "ch0 equal on only 2 lines: no trip (fails if the window shrinks to 2)")

n, msg = run([status(10, 1, 700), status(15, 2, 700), status(20, 3, 700)])
check(n == 2 and "on ch1" in msg, "ch1 alone stuck: trips (fails if only ch0 is watched)")

n, msg = run([status(10, None, 700), status(15, None, 700), status(20, None, 700)])
check(n == 2 and "on ch1," in msg, "a ch0 section without played does not borrow ch1's (fails if [^ ;] widens to .)")

n, _ = run([status(10, 500, 1), "INFOR: UE TX_BANK_STATUS=ch0:played=500", status(15, 500, 2),
            "INFOR: UE TX_BANK_STATUS=ch0:played=500", status(20, 501, 3)])
check(n is None, "lines without 'TX status:' are not counted (fails if the TX status filter goes)")

n, msg = run(["09:000001 ERROR: tx_stream[1] PLAYOUT FROZEN: played unchanged for 1 s"])
check(n == 0 and "device detector" in msg, "the device's PLAYOUT FROZEN line trips at once (fails if that check goes)")

print("%d failure(s)" % fails)
sys.exit(1 if fails else 0)
