# Known-answer checks for the dashboard's new parsers (MER/EVM, delay spread, CIR1, MET1).
# Stdlib only; run from csi_gui/ (ctest does). AP-79.
import math, random, struct, sys
sys.argv = ["x"]
import importlib.util
spec = importlib.util.spec_from_file_location("cs", "csi_server.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)
# MER: QPSK at a known SNR (complex Gaussian error of power 10^(-snr/10) on unit-power points)
random.seed(1)
ref = cs._ideal(2)
for snr in (15.0, 26.0, 40.0):
    s = math.sqrt(10 ** (-snr / 10) / 2)
    pts = []
    for i in range(20000):
        a, b = random.choice(ref); pts.append([a + random.gauss(0, s), b + random.gauss(0, s)])
    evm, mer = cs._mer(pts, 2)
    check(abs(mer - snr) < 0.3, "QPSK at %.0f dB SNR reads MER %.2f dB, EVM %.2f %%" % (snr, mer, evm))
check(abs(cs._mer([[p[0], p[1]] for p in ref] * 4, 2)[1] - 99.0) < 1e-9, "perfect points read 99 dB (no division by zero)")
check(cs._mer([[1, 0]] * 3, 2) is None, "fewer than 8 points: no figure")
# 16-QAM alphabet has unit average power
r16 = cs._ideal(4); check(abs(sum(a*a+b*b for a, b in r16) / 16 - 1) < 1e-12, "16-QAM ideal alphabet is unit power")
# Delay spread: two equal taps 10 taps apart -> 5 taps rms
db = [-60.0] * 64; db[16] = 0.0; db[26] = 0.0
check(abs(cs._delay_spread_ns(db, 8.0) - 40.0) < 1e-9, "two equal taps 10 apart at 8 ns: rms spread 40 ns")
db2 = [-60.0] * 64; db2[16] = 0.0
check(cs._delay_spread_ns(db2, 8.0) == 0.0, "one tap: 0 ns")
db3 = [-60.0] * 64; db3[16] = 0.0; db3[40] = -25.0
check(cs._delay_spread_ns(db3, 8.0) == 0.0, "a tap under the -20 dB floor is ignored")
# Wire round trips
cir = struct.pack("<IIIIIIIf", cs.MAGIC_CIR, 7, 1, 4, 1, 99, 4096, 8.138) + struct.pack("<4f", -30, 0, -6, -40)
a, rec = cs._parse_cir(cir)
check(a == 1 and rec["frame"] == 7 and rec["db"] == [-30, 0, -6, -40] and rec["peak"] == 99 and abs(rec["tap_ns"] - 8.138) < 1e-4, "CIR1 parses")
met = struct.pack("<IIIIIddd", cs.MAGIC_MET, 1, 2, 4096, 1596, 4380e6, 30e3, 47.88e6)
a, m = cs._parse_met(met)
check(a == 1 and m["ch"] == "C" and m["fc_mhz"] == 4380.0 and m["bw_mhz"] == 47.88 and m["fft"] == 4096, "MET1 parses (channel C, 4380 MHz, 47.88 MHz)")
check(cs._parse_met(met[:-8]) is None, "a short MET1 is dropped")
print("FAILED %d" % fails if fails else "ALL PASS"); sys.exit(1 if fails else 0)
