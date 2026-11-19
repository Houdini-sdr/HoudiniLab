# The dashboard's teardown (teardown_framer.teardown_node over beacon_tdd's
# _teardown) runs abort, TDD_CMD tx_clear and the gate release, and takes the
# device's own verdict on the clear: tx_clear raises by name when the device
# refuses it or judges a bank not cleared. A fake device stands in; no
# SoapyHoudiniSDR checkout is needed. Run from csi_gui/ (ctest does).
import os, sys, types
here = os.path.dirname(os.path.abspath(__file__))
# beacon_tdd's module-level imports, stood in: only its teardown runs here
for name in ("SoapySDR", "numpy", "houdini_setup"):
    sys.modules[name] = types.ModuleType(name)
for k in ("SOAPY_SDR_RX", "SOAPY_SDR_TX", "SOAPY_SDR_HAS_TIME", "SOAPY_SDR_END_BURST"):
    setattr(sys.modules["SoapySDR"], k, 0)
for k in ("run_burst", "rx_framing", "iq_from_cs16", "tx_iq_tone", "tx_lfm_chirp"):
    setattr(sys.modules["houdini_setup"], k, None)
sys.path[:0] = [os.path.join(here, "..", "tests", "hil"), here]
import beacon_tdd as bt
import teardown_framer as tf
tf.SOAPY_SDR_RX = 0
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what, flush=True); fails += (not ok)

# The device's render of two idle TX banks: it carries no count of clears.
BANK = ("acked=0,late=0,under=0,seqerr=0,zerofill=0,fill=0,hwm=0,drops=0,last_seq=0,active=0,cap=64,gated=0,smiss=0,"
        "aclose=0,parked=0,late_seen=0,efault=0,played=0,clear_drops=0,clear_busy=0,tick=100:130,malformed=0,"
        "cancels=0,flushed=0,clock_aborts=0")

class Fake:
    """A device whose TDD_CMD tx_clear lands, unless `raises` names the command
    that raises instead: 'tx_clear' as the device's refusal or its verdict that
    a bank did not clear, 'abort' or 'gate_release' as a failed write. Every
    read is counted."""
    def __init__(self, raises=None):
        self.raises, self.log, self.reads = raises, [], 0
    def readSetting(self, k):
        self.reads += 1
        return "ch0:%s;ch1:%s" % (BANK, BANK) if k == "TX_BANK_STATUS" else "accepted=1"
    def writeSetting(self, k, v):
        self.log.append("%s=%s" % (k, v))
        if k == "TDD_CMD" and v == self.raises:
            raise RuntimeError("TDD_CMD %s: TX ch1 clear_busy did not fall within 1 ms" % v if v == "tx_clear"
                               else "TDD_CMD %s: SoapyRPCUnpacker::recv() TIMEOUT" % v)
    def getSampleRate(self, d, c): return 1.0
    def setSampleRate(self, d, c, r): pass

STEPS = ["TDD_CMD=abort", "TDD_CMD=tx_clear", "TDD_CMD=gate_release"]

f = Fake()
check(bt._teardown(f) is True and f.log == STEPS,
      "a clear that lands: True, the three steps written in the ladder's order (mutation: the gate released before "
      "the clear, which lets a strobe play the uncleared bank: %s)" % f.log)
check(f.reads == 0,
      "the teardown takes the device's verdict and reads nothing (mutation: the clear-epoch second check restored, "
      "which finds no epoch in the device's render and reports every teardown unchecked: %d reads)" % f.reads)
f = Fake(raises="tx_clear")
check(bt._teardown(f) is False and f.log == STEPS,
      "a tx_clear the device refuses or judges not landed: False, and the gate release is still written "
      "(mutations: the clear's raise swallowed, which returns True over uncleared banks; the raise inside the "
      "writes' try, which skips the gate release: %s)" % f.log)
f = Fake(raises="abort")
check(bt._teardown(f) is False and f.log == STEPS[:1],
      "an abort that raises: False, and nothing is cleared over a framer that may still run (mutation: the abort's "
      "raise ignored: %s)" % f.log)
f = Fake(raises="gate_release")
check(bt._teardown(f) is False,
      "a gate release that raises: False (mutation: only the clear's result returned)")
hs = types.SimpleNamespace(open_device=lambda **kw: {"sdr": Fake(raises="tx_clear")})
check(tf.teardown_node(hs, bt._teardown, "192.0.2.1", 1, 2) is False,
      "teardown_node reports a clear that did not land as FAILED (mutation: _teardown's result ignored)")
hs = types.SimpleNamespace(open_device=lambda **kw: {"sdr": Fake()})
check(tf.teardown_node(hs, bt._teardown, "192.0.2.1", 1, 2) is True, "teardown_node: a clean teardown is torn down")
def no_open(**kw):
    raise RuntimeError("no route")
check(tf.teardown_node(types.SimpleNamespace(open_device=no_open), bt._teardown, "192.0.2.1", 1, 2) is False,
      "teardown_node: a radio that does not open is FAILED (mutation: an unopened radio counted as torn down)")
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
