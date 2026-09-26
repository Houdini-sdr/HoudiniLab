# Known-answer checks for the dashboard's new parsers (MER/EVM, delay spread, CIR1, MET1, SPC1).
# Stdlib only; run from csi_gui/ (ctest does). AP-79.
import math, random, re, struct, sys
sys.argv = ["x"]
import importlib.util
spec = importlib.util.spec_from_file_location("cs", "csi_server.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)
def ideal(mod):
    """The square-QAM alphabet at unit average power, as the page draws it."""
    lvl = int(round(math.sqrt(2 ** mod)))
    lv = [-(lvl - 1) + 2 * i for i in range(lvl)]
    nrm = math.sqrt(sum(a * a + b * b for a in lv for b in lv) / (lvl * lvl))
    return [(a / nrm, b / nrm) for a in lv for b in lv]
def mer(pts, mod):
    """One record's (EVM %, MER dB) through the dashboard's own two steps."""
    r = cs._mer_err(pts, mod)
    return None if r is None else cs._evm_mer(r[0])
def noisy(ref, snr, n):
    s = math.sqrt(10 ** (-snr / 10) / 2)
    out = []
    for i in range(n):
        a, b = random.choice(ref); out.append([a + random.gauss(0, s), b + random.gauss(0, s)])
    return out
# MER: QPSK at a known SNR (complex Gaussian error of power 10^(-snr/10) on unit-power points)
random.seed(1)
ref = ideal(2)
for snr in (15.0, 26.0, 40.0):
    evm, m = mer(noisy(ref, snr, 20000), 2)
    check(abs(m - snr) < 0.3, "QPSK at %.0f dB SNR reads MER %.2f dB, EVM %.2f %%" % (snr, m, evm))
# Fails under: dropping the 1e-10 floor in _evm_mer (log10 of 0 raises).
check(abs(mer([[p[0], p[1]] for p in ref] * 4, 2)[1] - 100.0) < 1e-9, "perfect points read 100 dB (the error-power floor, no division by zero)")
check(cs._mer_err([[1, 0]] * 3, 2) is None, "fewer than 8 points: no figure")
# 16-QAM alphabet has unit average power
r16 = ideal(4); check(abs(sum(a*a+b*b for a, b in r16) / 16 - 1) < 1e-12, "16-QAM ideal alphabet is unit power")
# The O(1) slicer equals a brute-force nearest-point search, every alphabet
for mod in (2, 4, 6):
    ref = ideal(mod); ok = True
    for _ in range(3000):
        x, y = random.uniform(-1.6, 1.6), random.uniform(-1.6, 1.6)
        brute = min((x - a) ** 2 + (y - b) ** 2 for a, b in ref)
        e, _n = cs._mer_err([[x, y]] * 8, mod)
        ok = ok and abs(e - brute) < 1e-9
    check(ok, "the per-axis slicer equals the brute-force nearest point, mod %d (mutation: round without the clamp)" % mod)
# 16-QAM at a known SNR
m16 = mer(noisy(ideal(4), 26.0, 20000), 4)[1]
check(abs(m16 - 26.0) < 0.3, "16-QAM at 26 dB SNR reads MER %.2f dB" % m16)
# Averaging pools ERROR POWER over the window, weighted by points
cs._mer_hist.clear()
cs._mer_avg(9, 0.01, 100, 0.0)
e, n = cs._mer_avg(9, 0.0001, 300, 0.5)
check(abs(e - (0.01 * 100 + 0.0001 * 300) / 400) < 1e-12 and n == 400, "records inside 1 s pool their error power by point count (mutation: average the dB)")
e, n = cs._mer_avg(9, 0.0001, 300, 2.0)
check(n == 300, "records older than 1 s drop out, both of them (mutation: never pop the old entries, or pop only one)")
# The window is per antenna: the X-band card must not pool the sub-6 card's error.
cs._mer_hist.clear()
cs._mer_avg(1, 0.01, 100, 0.0)
e2, n2 = cs._mer_avg(2, 0.0001, 300, 0.1)
e1, n1 = cs._mer_avg(1, 0.01, 100, 0.2)
check(abs(e2 - 0.0001) < 1e-15 and n2 == 300 and abs(e1 - 0.01) < 1e-15 and n1 == 200,
      "each antenna pools only its own records (mutation: key the history to antenna 0 for every card)")
# EVM is the rms error in percent, 100 sqrt(e) (ETSI TR 101 290), next to MER -10 log10(e).
evm, m = cs._evm_mer(0.01)
check(abs(evm - 10.0) < 1e-12 and abs(m - 20.0) < 1e-12,
      "an error power of 0.01 reads EVM 10 %% and MER 20 dB (EVM %.4f %%, MER %.4f dB) (mutation: EVM as 100 e, which reads 1 %%)"
      % (evm, m))
evm, m = cs._evm_mer(0.0004)
check(abs(evm - 2.0) < 1e-12 and abs(m - 33.9794) < 1e-4,
      "an error power of 4e-4 reads EVM 2 % and MER 33.98 dB (mutation: EVM as 100 e, which reads 0.04 %)")
# The page's figure end to end: two CNS1 records of one antenna, 20 and 40 dB,
# parsed back to back, report their POOLED error, not the last record's own.
cs._mer_hist.clear()
def cns1(frame, ant, pts):
    return cs.CNS_HDR.pack(cs.MAGIC_CNS, frame, ant, len(pts), 2) + struct.pack("<%df" % (2 * len(pts)), *[v for p in pts for v in p])
p20, p40 = noisy(ideal(2), 20.0, 2000), noisy(ideal(2), 40.0, 2000)
cs._parse_cns(cns1(1, 5, p20))
a, rec = cs._parse_cns(cns1(2, 5, p40))
pooled = -10 * math.log10((cs._mer_err(p20, 2)[0] + cs._mer_err(p40, 2)[0]) / 2)
# Fails under: _parse_cns converting the record's own error (skipping _mer_avg).
check(a == 5 and abs(rec["mer_db"] - pooled) < 0.051 and rec["mer_pts"] == 4000 and len(rec["pts"]) == 2000,
      "CNS1 reports the 1 s pooled MER %.1f dB over %d points (pooled truth %.2f; the 40 dB record alone reads ~40)"
      % (rec["mer_db"], rec["mer_pts"], pooled))
# Two antennas' records interleaved, as the two lanes arrive: each card reads
# its own lane, a 20 dB lane and a 40 dB lane, not their pool (about 23 dB).
cs._mer_hist.clear()
cs._parse_cns(cns1(1, 0, p20)); cs._parse_cns(cns1(1, 2, p40))
_, r0 = cs._parse_cns(cns1(2, 0, p20)); _, r2 = cs._parse_cns(cns1(2, 2, p40))
own20 = -10 * math.log10(cs._mer_err(p20, 2)[0]); own40 = -10 * math.log10(cs._mer_err(p40, 2)[0])
check(abs(r0["mer_db"] - own20) < 0.051 and abs(r2["mer_db"] - own40) < 0.051 and r0["mer_pts"] == r2["mer_pts"] == 4000,
      "interleaved lanes each report their own MER, %.1f and %.1f dB (truth %.2f and %.2f) "
      "(mutation: key the history to antenna 0 for every card)" % (r0["mer_db"], r2["mer_db"], own20, own40))
# The record's EVM end to end, a known answer: unit-power QPSK points each
# displaced by 0.1 in I have an error power of exactly 0.01, so EVM 10 % and MER 20 dB.
s = 1 / math.sqrt(2)
qp = [[a * s + 0.1, b * s] for a in (-1, 1) for b in (-1, 1)] * 4
_, rq = cs._parse_cns(cns1(1, 7, qp))
check(rq["evm_pct"] == 10.0 and rq["mer_db"] == 20.0 and rq["mer_pts"] == 16,
      "a CNS1 record with rms error 0.1 reports evm_pct 10.0 and mer_db 20.0 (read %r and %r) "
      "(mutation: EVM as 100 e, which reads 1.0)" % (rq["evm_pct"], rq["mer_db"]))
# Delay statistics, the excess delays from the first tap above the threshold
pre = 64
db = [-60.0] * 128; db[pre] = 0.0
d = cs._delay_stats(db, 8.0)
check(d["rms_ns"] == 0.0 and d["mean_ns"] == 0.0 and d["max_ns"] == 0.0 and d["thr_db"] == -20.0, "one tap: 0 ns everywhere, threshold -20 dB")
db = [-60.0] * 128; db[pre] = 0.0; db[pre + 10] = 0.0
d = cs._delay_stats(db, 8.0)
check(abs(d["rms_ns"] - 40.0) < 1e-9 and abs(d["mean_ns"] - 40.0) < 1e-9 and abs(d["max_ns"] - 80.0) < 1e-9,
      "two equal taps 10 apart at 8 ns: rms 40, mean excess 40, max excess 80 ns")
# A -3 dB path arriving 10 taps BEFORE the strongest: the standard figures count
# from that first arrival (mean 10/(1+10^-0.3) = 6.66 taps = 53.3 ns, max 80 ns), not
# from the peak (which read -26.7 and 0 ns).
db = [-60.0] * 128; db[pre] = 0.0; db[pre - 10] = -3.0
d = cs._delay_stats(db, 8.0)
# Fails under: measuring the excess delays from the strongest tap (the old `pre`).
check(abs(d["mean_ns"] - 53.3) < 0.051 and abs(d["max_ns"] - 80.0) < 1e-9,
      "an earlier weaker path: mean excess %.1f, max excess %.1f ns from the first arrival" % (d["mean_ns"], d["max_ns"]))
db = [-15.0] * 128; db[pre] = 0.0; db[pre + 20] = -12.0
d = cs._delay_stats(db, 8.0)
check(d["thr_db"] == -9.0 and d["max_ns"] == 0.0, "a -15 dB floor lifts the threshold to -9 dB, so a -12 dB tap is not counted (mutation: fixed -20 dB)")
# A lone path through the Hann window houdini/cir.h applies (R2's 96 of 256 tones
# at 122.88 MSPS, B = 46.08 MHz): the excess delays read the mainlobe itself,
# about 1.5/B mean and 3/B max, which is what the page states as the floors.
N, lo, hi = 256, 80, 175
span = hi - lo + 1
X = [0j] * N
for k in range(N):
    kc = (k + N // 2) % N
    if lo <= kc <= hi:
        X[k] = 0.5 - 0.5 * math.cos(2 * math.pi * (kc - lo + 0.5) / span)
p = [abs(sum(X[k] * complex(math.cos(2 * math.pi * k * t / N), math.sin(2 * math.pi * k * t / N))
             for k in range(N))) ** 2 for t in range(N)]
pk = max(range(N), key=lambda t: p[t])
db = [max(-60.0, 10 * math.log10(max(p[(pk - pre + i) % N], 1e-30) / p[pk])) for i in range(128)]
b_mhz = span * 122.88 / N
d = cs._delay_stats(db, 1e3 / 122.88)
# Fails under: the excess delays measured from the strongest tap (a lone path
# then reads a mean of about 0 and a max of about 1.5/B), or the spread's sqrt
# dropped (rms then reads about 0.87/B).
check(0.4e3 / b_mhz < d["rms_ns"] < 0.7e3 / b_mhz and 1.2e3 / b_mhz < d["mean_ns"] < 1.8e3 / b_mhz
      and 2.5e3 / b_mhz < d["max_ns"] < 3.5e3 / b_mhz,
      "a lone Hann-windowed path reads rms %.1f, mean %.1f and max %.1f ns: the page's 0.5/B, 1.5/B and 3/B at B %.2f MHz"
      % (d["rms_ns"], d["mean_ns"], d["max_ns"], b_mhz))
# Fails under: the page's hint stating other floors than the ones measured
# above, or pairing a floor with the wrong label (rms and mean swapped).
check(all(re.search(r"%s '\s*\+\(%s/m\.bw_mhz\)" % (lab, val), cs.PAGE)
          for lab, val in (("rms", r"0\.5e3"), ("mean", r"1\.5e3"), ("max", r"3e3"))),
      "the page states the measured floors, each under its own label (rms 0.5/B, mean 1.5/B, max 3/B)")
# Wire round trips
cir = struct.pack("<IIIIIIIf", cs.MAGIC_CIR, 7, 1, 4, 1, 99, 4096, 8.138) + struct.pack("<4f", -30, 0, -6, -40)
a, rec = cs._parse_cir(cir)
check(a == 1 and rec["frame"] == 7 and rec["db"] == [-30, 0, -6, -40] and rec["peak"] == 99 and abs(rec["tap_ns"] - 8.138) < 1e-4, "CIR1 parses")
met = struct.pack("<IIIIIddd", cs.MAGIC_MET, 1, 2, 4096, 1596, 4380e6, 30e3, 47.88e6)
a, m = cs._parse_met(met)
check(a == 1 and m["ch"] == "C" and m["fc_mhz"] == 4380.0 and m["bw_mhz"] == 47.88 and m["fft"] == 4096, "MET1 parses (channel C, 4380 MHz, 47.88 MHz)")
check(cs._parse_met(met[:-8]) is None, "a short MET1 is dropped")
# SPC1: the pilot spectrum, [magic][frame][ant][nbins][nfft][rate f32][rbw_hz f32] + nbins dBFS f32
def spc1(nbins=512, nfft=2048, rate=122.88e6, db=None, magic=None):
    db = [-57.0 - 0.1 * (i % 7) for i in range(nbins)] if db is None else db
    return struct.pack("<IIIIIff", cs.MAGIC_SPC if magic is None else magic, 42, 1, nbins, nfft, rate,
                       rate / max(nbins, 1)) + struct.pack("<%df" % len(db), *db)
check(struct.pack("<I", cs.MAGIC_SPC) == b"1CPS" and cs.MAGIC_SPC == 0x53504331,
      "SPC1's magic is the sounder's 0x53504331 (mutation: any other constant)")
a, sp = cs._parse_spc(spc1())
check(a == 1 and sp["frame"] == 42 and sp["nbins"] == 512 and sp["nfft"] == 2048 and len(sp["db"]) == 512
      and sp["db"][0] == -57.0 and sp["db"][6] == -57.6 and abs(sp["rbw_hz"] - 240e3) < 0.1 and sp["rate"] == 122.88e6,
      "SPC1 parses: 512 bins of dBFS in order, 240 kHz bins at 122.88 Msps (mutation: read the values from byte 24)")
check(cs._parse_spc(spc1()[:-4]) is None, "an SPC1 one value short is dropped, not raised (the unpack would overrun)")
check(cs._parse_spc(spc1()[:20]) is None, "an SPC1 shorter than its header is dropped, not raised")
check(cs._parse_spc(spc1() + b"\0\0\0\0") is None, "an SPC1 with trailing bytes is dropped (mutation: a >= length check)")
check(cs._parse_spc(spc1(nbins=0, db=[])) is None, "nbins 0 is dropped (mutation: drop the nbins check, the page divides by it)")
check(cs._parse_spc(spc1(nbins=500, db=[-60.0] * 500)) is None,
      "nfft not a multiple of nbins is dropped (mutation: drop the nfft % nbins check, the page's bin centres go wrong)")
check(cs._parse_spc(spc1(db=[-60.0] * 511 + [float("nan")])) is None, "a NaN bin is dropped: it would break the page's JSON")
check(cs._parse_spc(spc1(rate=float("nan"))) is None and cs._parse_spc(spc1(rate=0.0)) is None,
      "a NaN or zero rate is dropped (the page divides by it)")
check(re.search(r"data-view=\"adc\">Spectrum<", cs.PAGE) is not None and "function drawSpc(" in cs.PAGE
      and "if(rec.spc && rec.spc.frame!==card.lastSpc)" in cs.PAGE,
      "the page has the Spectrum tab and draws SPC1 records (mutation: onData never reads rec.spc)")
# The |H| axis top: --mag-top, else the config's dashboard_mag_top, else the default.
import json, os, tempfile
td = tempfile.mkdtemp(prefix="csi_magtop_")
for name, body in (("with.json", {"dashboard_mag_top": 115}), ("without.json", {}),
                   ("bad.json", {"dashboard_mag_top": "115"}), ("flag.json", {"dashboard_mag_top": True})):
    json.dump(body, open(os.path.join(td, name), "w"))
check(cs._mag_top(td, "with.json", None) == 115.0, "the config's dashboard_mag_top sets the |H| axis top (mutation: ignore the config)")
check(cs._mag_top(td, "with.json", 100.0) == 100.0, "an explicit --mag-top beats the config (mutation: config first)")
check(cs._mag_top(td, "without.json", None) == cs.MAG_TOP_DEFAULT == 90.0, "no key: the old default 90 (mutation: a new default)")
check(cs._mag_top(td, "bad.json", None) == 90.0 and cs._mag_top(td, "flag.json", None) == 90.0,
      "a non-number (a string, a bool) falls back to the default (mutation: float() any value)")
check(cs._mag_top(td, "missing.json", None) == 90.0, "an unreadable config falls back to the default")
demo = json.load(open(os.path.join(os.path.dirname(os.path.abspath("csi_server.py")), "..", "files", "houdini-dualband.json")))
check(demo.get("dashboard_mag_top") == 115, "the demo config carries its axis top (|H| read 97-104 dB there)")
import shutil; shutil.rmtree(td, ignore_errors=True)  # no temp dir left per run
print("FAILED %d" % fails if fails else "ALL PASS"); sys.exit(1 if fails else 0)
