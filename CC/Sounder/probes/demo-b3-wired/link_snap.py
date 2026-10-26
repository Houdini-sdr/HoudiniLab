# The 100G link's record on each node (FHCORE, register map v3) and the rig host's PHY link-down counts, read with no
# stream open, before and after a run (HS-334):
#   python3 link_snap.py <label> <node> [<node> ...] [--clear-cause]
# with the release's prefix in SOAPY_SDR_ROOT and SOAPY_SDR_PLUGIN_PATH empty. The order is the fpga lane's, the
# record first, before anything can reset the core. --clear-cause writes LINK_CAUSE after reading it, so its [31]
# reading 1 afterwards means a fall during the run, not before it.
import subprocess, sys, time
import SoapySDR

REGS = (("LINK_CAUSE", 0x50), ("LINK_OUTAGE", 0x54), ("LINK_AGE", 0x58), ("LINK_STATUS", 0x40),
        ("LINK_EVENTS", 0x44), ("RSFEC_CORRECTED", 0xB0), ("RSFEC_UNCORRECTED", 0xB4))
PORTS = ("enp1s0f0np0", "enp1s0f1np1")  # burton's data ports, to .21 and .22
label, clear = sys.argv[1], "--clear-cause" in sys.argv
nodes = [a for a in sys.argv[2:] if not a.startswith("--")]
print("%s UTC %s" % (label, time.strftime("%FT%TZ", time.gmtime())))
for ip in nodes:
    sdr = SoapySDR.Device({"driver": "houdinisdr", "remote": "tcp://%s:55132" % ip,
                           "remote:driver": "houdinisdr-device", "remote:type": "houdinisdr", "timeout": "3000000"})
    try:
        for name, off in REGS:
            try:
                print("%s %s FHCORE %s (0x%02X) = 0x%08X" % (label, ip, name, off, sdr.readRegister("FHCORE", off)))
            except Exception as e:
                print("%s %s FHCORE %s READ FAILED %s" % (label, ip, name, e))
        if clear:
            try:
                sdr.writeRegister("FHCORE", 0x50, 0)
                print("%s %s LINK_CAUSE written, reads 0x%08X" % (label, ip, sdr.readRegister("FHCORE", 0x50)))
            except Exception as e:
                print("%s %s LINK_CAUSE WRITE FAILED %s" % (label, ip, e))
    finally:
        sdr.close()
for port in PORTS:
    out = subprocess.run(["ethtool", "-S", port], capture_output=True, text=True).stdout
    line = [l.strip() for l in out.splitlines() if "link_down_events_phy" in l]
    print("%s burton %s %s" % (label, port, line[0] if line else "link_down_events_phy NOT FOUND"))
