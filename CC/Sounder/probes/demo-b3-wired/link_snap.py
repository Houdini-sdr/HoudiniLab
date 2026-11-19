# The 100G link's record on each node and the rig host's PHY link-down counts, read with no stream open, before and
# after a run (HS-334):
#   python3 link_snap.py <label> <node> [<node> ...] [--clear-cause]
# with the release's prefix in SOAPY_SDR_ROOT and SOAPY_SDR_PLUGIN_PATH empty. The order is the fpga lane's, the
# record (LINK_FALL) first, before anything can reset the core. Both are read as settings, which the device resolves
# on its own register map. --clear-cause clears the record after reading it, so its valid=1 afterwards means a fall
# during the run, not before it: the one raw write here, of LINK_CAUSE at the offset SoapyHoudiniSDR's
# HOUDINI_PROTOCOL section 2.7 gives for it. That offset belongs to one register map: check it there before use.
import subprocess, sys, time
import SoapySDR

KEYS = ("LINK_FALL", "LINK_STATUS")
LINK_CAUSE = 0x08C
PORTS = ("enp1s0f0np0", "enp1s0f1np1")  # burton's data ports, to .21 and .22
label, clear = sys.argv[1], "--clear-cause" in sys.argv
nodes = [a for a in sys.argv[2:] if not a.startswith("--")]
print("%s UTC %s" % (label, time.strftime("%FT%TZ", time.gmtime())))
for ip in nodes:
    sdr = SoapySDR.Device({"driver": "houdinisdr", "remote": "tcp://%s:55132" % ip,
                           "remote:driver": "houdinisdr-device", "remote:type": "houdinisdr", "timeout": "3000000"})
    try:
        for key in KEYS:
            try:
                print("%s %s %s = %s" % (label, ip, key, sdr.readSetting(key)))
            except Exception as e:
                print("%s %s %s READ FAILED %s" % (label, ip, key, e))
        if clear:
            try:
                sdr.writeRegister("FHCORE", LINK_CAUSE, 0)
                print("%s %s LINK_CAUSE written, LINK_FALL = %s" % (label, ip, sdr.readSetting("LINK_FALL")))
            except Exception as e:
                print("%s %s LINK_CAUSE WRITE FAILED %s" % (label, ip, e))
    finally:
        sdr.close()
for port in PORTS:
    out = subprocess.run(["ethtool", "-S", port], capture_output=True, text=True).stdout
    line = [l.strip() for l in out.splitlines() if "link_down_events_phy" in l]
    print("%s burton %s %s" % (label, port, line[0] if line else "link_down_events_phy NOT FOUND"))
