# The dashboard's teardown (teardown_framer.teardown_node over beacon_tdd's
# _teardown) runs TDD_CMD tx_clear and checks the clear epoch it must move on
# every TX bank: a fake device answers with a captured TX_BANK_STATUS, parsed by the
# driver's own setting_reports from the SoapyHoudiniSDR host examples. Skipped
# (77) where those examples are not checked out. Run from csi_gui/ (ctest does).
import os, re, sys, types
here = os.path.dirname(os.path.abspath(__file__))
examples = os.environ.get("HOUDINI_EXAMPLES", os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if not os.path.exists(os.path.join(examples, "setting_reports.py")):
    print("SKIP no setting_reports.py in %s (set HOUDINI_EXAMPLES)" % examples); sys.exit(77)
fixture = open(os.path.join(here, "..", "tests", "comms-func", "fixtures", "link_health", "stack21_b3_before.txt")).read()
CAPTURED = re.search(r"^TX_BANK_STATUS = (.*)$", fixture, re.M).group(1)  # .21's two TX banks, fpga 1.47
# beacon_tdd's module-level imports, stood in: only its teardown runs here
for name in ("SoapySDR", "numpy", "houdini_setup"):
    sys.modules[name] = types.ModuleType(name)
for k in ("SOAPY_SDR_RX", "SOAPY_SDR_TX", "SOAPY_SDR_HAS_TIME", "SOAPY_SDR_END_BURST"):
    setattr(sys.modules["SoapySDR"], k, 0)
for k in ("run_burst", "rx_framing", "iq_from_cs16", "tx_iq_tone", "tx_lfm_chirp"):
    setattr(sys.modules["houdini_setup"], k, None)
sys.path[:0] = [examples, os.path.join(here, "..", "tests", "hil"), here]
import beacon_tdd as bt
import teardown_framer as tf
import setting_reports as sr
tf.SOAPY_SDR_RX = 0
# The capture is fpga 1.47's 28 keys, cancels (REPLAY_CANCELS) the last; a
# parser from before SoapyHoudiniSDR 0.7.0 takes exactly the 27 before it and
# refuses the rest, so for one that key is dropped (AP-125; it goes when 0.7.0
# is the floor); any other difference still fails the parse below.
if "cancels" not in sr.TX_BANK_KEYS:
    CAPTURED = re.sub(r",cancels=\d+", "", CAPTURED)
    print("NOTE the 1.47 capture's cancels is dropped for this pre-0.7.0 parser (%s)" % examples)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what, flush=True); fails += (not ok)

class Fake:
    """A device whose TDD_CMD tx_clear counts in both banks' clear epoch, unless
    it is refused (raises, nothing cleared) or returns without clearing (a
    device-side judge that passed a clear that never landed); `torn` reads
    after each clear are torn."""
    def __init__(self, lands=True, refused=False, torn=0, throws=False):
        self.lands, self.refused, self.tear, self.torn, self.throws = lands, refused, torn, 0, throws
        self.e, self.log = 587, []
    def readSetting(self, k):
        if k != "TX_BANK_STATUS":
            return "accepted=1"
        if self.throws:
            raise RuntimeError("SoapyRPCUnpacker type check FAIL")
        pair = "%d:%d" % (self.e - 1, self.e) if self.torn else "%d:%d" % (self.e, self.e)
        self.torn = max(0, self.torn - 1)
        return re.sub(r"epoch=\d+:\d+", "epoch=" + pair, CAPTURED)
    def writeSetting(self, k, v):
        self.log.append("%s=%s" % (k, v))
        if (k, v) == ("TDD_CMD", "tx_clear"):
            if self.refused:
                raise RuntimeError("TDD_CMD tx_clear: refused: the TDD framer is still engaged")
            if self.lands:
                self.e += 1
                self.torn = self.tear
    def getSampleRate(self, d, c): return 1.0
    def setSampleRate(self, d, c, r): pass

def ran_all(f):
    return all(x in f.log for x in ("TDD_CMD=abort", "TDD_CMD=tx_clear", "TDD_CMD=gate_release"))

f = Fake()
check(bt._clear_epochs(f) == {0: 587, 1: 587},
      "the capture parses to both banks' epochs through setting_reports (mutation: a parse that drops a bank or "
      "misreads the pair: %r)" % bt._clear_epochs(f))
f = Fake(lands=False); before = f.readSetting("TX_BANK_STATUS"); f.writeSetting("TDD_CMD", "tx_clear")
check(f.readSetting("TX_BANK_STATUS") == before,
      "the fake: a tx_clear that does not land leaves the epoch (an epoch that moves on any tx_clear proves nothing below)")
f = Fake()
check(bt._teardown(f) is True and ran_all(f), "a clear that lands: True, every step written")
f = Fake(lands=False)
check(bt._teardown(f) is False and ran_all(f),
      "a tx_clear accepted without clearing: False, after the gate release (mutation: the epoch check removed, "
      "which returns True over uncleared banks)")
f = Fake(refused=True)
check(bt._teardown(f) is False and ran_all(f),
      "a refused tx_clear: False, and the gate release is still written (mutation: the refusal inside the writes' "
      "try, which skips the gate release)")
f = Fake(torn=1)
check(bt._teardown(f) is True, "a read torn by the clear is read again (mutation: one read, which leaves it unchecked)")
f = Fake(torn=9)
check(bt._teardown(f) is True and ran_all(f),
      "every read torn: unchecked here, a warning and True (mutation: a torn read taken as an epoch)")
f = Fake(throws=True)
check(bt._teardown(f) is True and ran_all(f),
      "a TX_BANK_STATUS read that raises: the clear and the gate release are still written (mutation: the read "
      "inside the writes' try, which skips both)")
hs = types.SimpleNamespace(open_device=lambda **kw: {"sdr": Fake(lands=False)})
check(tf.teardown_node(hs, bt._teardown, "192.0.2.1", 1, 2) is False,
      "teardown_node reports a clear that did not land as FAILED (mutation: _teardown's result ignored)")
hs = types.SimpleNamespace(open_device=lambda **kw: {"sdr": Fake()})
check(tf.teardown_node(hs, bt._teardown, "192.0.2.1", 1, 2) is True, "teardown_node: a clean teardown is torn down")
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
