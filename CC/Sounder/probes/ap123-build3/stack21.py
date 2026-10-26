# .21's stack and link state, reads only: the identity, LINK_STATUS, EGRESS_STATUS, TX_BANK_STATUS, CLOCK_ADJ, and the
# FHCORE registers that name a link drop (HS-334: read them before any reset)
import sys, SoapySDR, time
d = SoapySDR.Device({"driver": "houdinisdr", "remote": "tcp://168.6.244.21:55132", "timeout": "3000000"})
hw = d.getHardwareInfo()
print("UTC", time.strftime("%FT%TZ", time.gmtime()))
for k in sorted(hw):
    if any(s in k for s in ("version", "commit", "build", "board", "sha", "xsa")):
        print("hw", k, "=", hw[k])
for k in ("LINK_STATUS", "EGRESS_STATUS", "TX_BANK_STATUS", "CLOCK_ADJ"):
    try:
        print(k, "=", d.readSetting(k))
    except Exception as e:
        print(k, "READ FAILED", e)
for name, off in (("LINK_CAUSE", 0x50), ("LINK_OUTAGE", 0x54), ("LINK_AGE", 0x58), ("RSFEC_CORRECTED", 0xB0),
                  ("RSFEC_UNCORRECTED", 0xB4)):
    try:
        print("FHCORE %s (0x%02X) = 0x%08X" % (name, off, d.readRegister("FHCORE", off)))
    except Exception as e:
        print("FHCORE", name, "READ FAILED", e)
