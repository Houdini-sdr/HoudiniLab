# The dashboard's teardown (teardown_framer.teardown_node over beacon_tdd's
# _teardown) judges its TX_CLEAR_ALL by the clear epoch it must move on every
# TX bank: a fake device answers with a captured TX_BANK_STATUS, parsed by the
# driver's own setting_reports from the SoapyHoudiniSDR host examples. Skipped
# (77) where those examples are not checked out. Run from csi_gui/ (ctest does).
import os, re, sys, types
here = os.path.dirname(os.path.abspath(__file__))
examples = os.environ.get("HOUDINI_EXAMPLES", os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if not os.path.exists(os.path.join(examples, "setting_reports.py")):
    print("SKIP no setting_reports.py in %s (set HOUDINI_EXAMPLES)" % examples); sys.exit(77)
fixture = open(os.path.join(here, "..", "tests", "comms-func", "fixtures", "link_health", "demo_stack_fi1.txt")).read()
CAPTURED = re.search(r"^TX_BANK_STATUS=(.*)$", fixture, re.M).group(1)  # BS .22's two TX banks
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
# The capture is fpga 1.45's 27 keys; SoapyHoudiniSDR 0.7.0's parser takes
# exactly 1.47's 28, which add cancels (REPLAY_CANCELS), and refuses the rest.
# Until a 1.47 capture replaces it, that one key is added at 0 for a parser
# that wants it (AP-125); any other difference still fails the parse below.
if "cancels" in sr.TX_BANK_KEYS and ",cancels=" not in CAPTURED:
    CAPTURED = ";".join(bank + ",cancels=0" for bank in CAPTURED.split(";"))
    print("NOTE the 1.45 capture is given cancels=0 for this 0.7.0 parser (%s)" % examples)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what, flush=True); fails += (not ok)

class Fake:
    """A device that takes TX_CLEAR_ALL at `at` only (a write anywhere else is
    ignored, as the fabric ignores an unmapped offset), counting it in both
    banks' clear epoch; `torn` reads after each clear are torn."""
    def __init__(self, at, epoch=587, counts=True, torn=0, throws=False):
        self.at, self.e, self.counts, self.tear, self.torn, self.throws, self.log = at, epoch, counts, torn, 0, throws, []
    def readSetting(self, k):
        if k != "TX_BANK_STATUS":
            return "accepted=1"
        if self.throws:
            raise RuntimeError("SoapyRPCUnpacker type check FAIL")
        pair = "%d:%d" % (self.e - 1, self.e) if self.torn else "%d:%d" % (self.e, self.e)
        self.torn = max(0, self.torn - 1)
        return re.sub(r"epoch=\d+:\d+", "epoch=" + pair, CAPTURED)
    def writeSetting(self, k, v): self.log.append("%s=%s" % (k, v))
    def writeRegister(self, core, off, v):
        self.log.append("reg %#x=%d" % (off, v))
        if core == "RFCORE" and off == self.at and v == 1:
            if self.counts:
                self.e += 1
            self.torn = self.tear
    def getSampleRate(self, d, c): return 1.0
    def setSampleRate(self, d, c, r): pass

R = bt.REG_TX_CLEAR_ALL
other = 0x084 if R == 0x24 else 0x24  # the other register map's offset (v3 moved it)
def ran_all(f):
    return "TDD_CMD=abort" in f.log and "reg %#x=1" % R in f.log and "TDD_CMD=gate_release" in f.log

f = Fake(R)
check(bt._clear_epochs(f) == {0: 587, 1: 587},
      "the capture parses to both banks' epochs through setting_reports (mutation: a parse that drops a bank or "
      "misreads the pair: %r)" % bt._clear_epochs(f))
f = Fake(other); before = f.readSetting("TX_BANK_STATUS"); f.writeRegister("RFCORE", R, 1)
check(f.readSetting("TX_BANK_STATUS") == before,
      "the fake ignores a clear at an offset it does not map (an epoch that moves on any write proves nothing below)")
f = Fake(R)
check(bt._teardown(f) is True and ran_all(f), "a clear that lands: True, every step written")
f = Fake(other)
check(bt._teardown(f) is False and ran_all(f),
      "a bitstream that maps TX_CLEAR_ALL elsewhere: False, after the gate release (mutation: the epoch check "
      "removed, which returns True over uncleared banks)")
f = Fake(R, torn=1)
check(bt._teardown(f) is True, "a read torn by the clear is read again (mutation: one read, which leaves it unchecked)")
f = Fake(R, torn=9)
check(bt._teardown(f) is True and ran_all(f),
      "every read torn: unchecked, a warning and True (mutation: a torn read taken as an epoch)")
f = Fake(R, throws=True)
check(bt._teardown(f) is True and ran_all(f),
      "a TX_BANK_STATUS read that raises: the clear and the gate release are still written (mutation: the read "
      "inside the writes' try, which skips both)")
f = Fake(R, epoch=0, counts=False)
check(bt._teardown(f) is True, "a bitstream without CLEAR_EPOCH (every bank 0): unchecked, not failed (mutation: a "
      "zero epoch judged as a miss, which fails every teardown on fpga 1.30)")
hs = types.SimpleNamespace(open_device=lambda **kw: {"sdr": Fake(other)})
check(tf.teardown_node(hs, bt._teardown, "192.0.2.1", 1, 2) is False,
      "teardown_node reports a clear that did not land as FAILED (mutation: _teardown's result ignored)")
hs = types.SimpleNamespace(open_device=lambda **kw: {"sdr": Fake(R)})
check(tf.teardown_node(hs, bt._teardown, "192.0.2.1", 1, 2) is True, "teardown_node: a clean teardown is torn down")
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
