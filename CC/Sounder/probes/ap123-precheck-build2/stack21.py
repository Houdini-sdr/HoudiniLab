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
