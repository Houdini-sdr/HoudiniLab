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
# The O(1) slicer equals a brute-force nearest-point search, every alphabet
for mod in (2, 4, 6):
    ref = cs._ideal(mod); ok = True
    for _ in range(3000):
        x, y = random.uniform(-1.6, 1.6), random.uniform(-1.6, 1.6)
        brute = min((x - a) ** 2 + (y - b) ** 2 for a, b in ref)
        e, _n = cs._mer_err([[x, y]] * 8, mod)
        ok = ok and abs(e - brute) < 1e-9
    check(ok, "the per-axis slicer equals the brute-force nearest point, mod %d (mutation: round without the clamp)" % mod)
# 16-QAM at a known SNR
s16 = math.sqrt(10 ** (-26 / 10) / 2); r16 = cs._ideal(4); p16 = []
for i in range(20000):
    a, b = random.choice(r16); p16.append([a + random.gauss(0, s16), b + random.gauss(0, s16)])
check(abs(cs._mer(p16, 4)[1] - 26.0) < 0.3, "16-QAM at 26 dB SNR reads MER %.2f dB" % cs._mer(p16, 4)[1])
# Averaging pools ERROR POWER over the window, weighted by points
cs._mer_hist.clear()
cs._mer_avg(9, 0.01, 100, 0.0)
e, n = cs._mer_avg(9, 0.0001, 300, 0.5)
check(abs(e - (0.01 * 100 + 0.0001 * 300) / 400) < 1e-12 and n == 400, "records inside 1 s pool their error power by point count (mutation: average the dB)")
e, n = cs._mer_avg(9, 0.0001, 300, 2.0)
check(n == 300, "records older than 1 s drop out")
# Delay statistics, relative to the peak at index `pre`
pre = 64
db = [-60.0] * 128; db[pre] = 0.0
d = cs._delay_stats(db, pre, 8.0)
check(d["rms_ns"] == 0.0 and d["mean_ns"] == 0.0 and d["max_ns"] == 0.0 and d["thr_db"] == -20.0, "one tap: 0 ns everywhere, threshold -20 dB")
db = [-60.0] * 128; db[pre] = 0.0; db[pre + 10] = 0.0
d = cs._delay_stats(db, pre, 8.0)
check(abs(d["rms_ns"] - 40.0) < 1e-9 and abs(d["mean_ns"] - 40.0) < 1e-9 and abs(d["max_ns"] - 80.0) < 1e-9,
      "two equal taps 10 apart at 8 ns: rms 40, mean excess 40, max excess 80 ns")
db = [-15.0] * 128; db[pre] = 0.0; db[pre + 20] = -12.0
d = cs._delay_stats(db, pre, 8.0)
check(d["thr_db"] == -9.0 and d["max_ns"] == 0.0, "a -15 dB floor lifts the threshold to -9 dB, so a -12 dB tap is not counted (mutation: fixed -20 dB)")
# Wire round trips
cir = struct.pack("<IIIIIIIf", cs.MAGIC_CIR, 7, 1, 4, 1, 99, 4096, 8.138) + struct.pack("<4f", -30, 0, -6, -40)
a, rec = cs._parse_cir(cir)
check(a == 1 and rec["frame"] == 7 and rec["db"] == [-30, 0, -6, -40] and rec["peak"] == 99 and abs(rec["tap_ns"] - 8.138) < 1e-4, "CIR1 parses")
met = struct.pack("<IIIIIddd", cs.MAGIC_MET, 1, 2, 4096, 1596, 4380e6, 30e3, 47.88e6)
a, m = cs._parse_met(met)
check(a == 1 and m["ch"] == "C" and m["fc_mhz"] == 4380.0 and m["bw_mhz"] == 47.88 and m["fft"] == 4096, "MET1 parses (channel C, 4380 MHz, 47.88 MHz)")
check(cs._parse_met(met[:-8]) is None, "a short MET1 is dropped")
print("FAILED %d" % fails if fails else "ALL PASS"); sys.exit(1 if fails else 0)
