# Offline validator for the two-stage beacon CFO estimator.
#
# Replicates receiver.cc estimateCFO() arithmetic exactly, on a synthetic
# beacon with the real geometry, to check the two-stage unwrap recovers a
# KNOWN injected offset and to find where it actually breaks.
import cmath, random, math
STS_LEN, STS_REPS, GOLD_LEN, GOLD_REPS = 16, 15, 128, 2
CORE = STS_LEN*STS_REPS + GOLD_LEN*GOLD_REPS
FS = 122.88e6
random.seed(7)

sts  = [complex(random.gauss(0,1), random.gauss(0,1)) for _ in range(STS_LEN)]
gold = [complex(random.gauss(0,1), random.gauss(0,1)) for _ in range(GOLD_LEN)]
beacon = sts*STS_REPS + gold*GOLD_REPS
assert len(beacon) == CORE == 496

def estimate(buf):
    g1 = STS_LEN*STS_REPS
    g2 = g1 + GOLD_LEN
    r_fine = sum(buf[g1+i].conjugate()*buf[g2+i] for i in range(GOLD_LEN))
    r_coarse = 0j
    for k in range(STS_REPS-1):
        for i in range(STS_LEN):
            r_coarse += buf[k*STS_LEN+i].conjugate()*buf[(k+1)*STS_LEN+i]
    f_fine   = cmath.phase(r_fine)  /(2*math.pi*GOLD_LEN)
    f_coarse = cmath.phase(r_coarse)/(2*math.pi*STS_LEN)
    amb = 1.0/GOLD_LEN
    m = round((f_coarse - f_fine)/amb)
    return f_fine + m*amb

def apply(f_hz, snr_db=None):
    fn = f_hz/FS
    out = [beacon[n]*cmath.exp(2j*math.pi*fn*n) for n in range(CORE)]
    if snr_db is not None:
        sp = sum(abs(x)**2 for x in out)/CORE
        np_ = sp/(10**(snr_db/10))
        s = math.sqrt(np_/2)
        out = [x+complex(random.gauss(0,s), random.gauss(0,s)) for x in out]
    return out

print("=== noiseless sweep: recovered vs injected ===")
for hz in [0, 100, 1e3, 1e4, 1e5, 3e5, 4.7e5, 4.79e5, 4.81e5, 6e5, 1e6, 3.8e6]:
    got = estimate(apply(hz))*FS
    ok = "ok" if abs(got-hz) < max(1.0, abs(hz)*1e-6) else "** WRAPPED **"
    print("  inject %+11.1f Hz -> %+11.1f Hz  %s" % (hz, got, ok))

print()
print("=== where the COMBINED estimator actually wraps ===")
# The fine stage alone is unambiguous only to fs/(2*GOLD_LEN) = 480 kHz, but the
# coarse stage resolves that ambiguity, so the combined range is the COARSE
# limit fs/(2*STS_LEN) = 3.84 MHz. Find the real edge rather than assuming it.
lo, hi = 1e5, 8e6
for _ in range(60):
    mid = (lo+hi)/2
    if abs(estimate(apply(mid))*FS - mid) < max(1.0, mid*1e-6): lo = mid
    else: hi = mid
print("  holds to      %.1f Hz" % lo)
print("  fine  alone   %.1f Hz  (fs/2/%d)" % (FS/2/GOLD_LEN, GOLD_LEN))
print("  coarse limit  %.1f Hz  (fs/2/%d)" % (FS/2/STS_LEN, STS_LEN))

print()
print("=== with noise, at the demo's measured ~48 dB beacon SNR ===")
for snr in [48, 30, 20, 10]:
    errs = [abs(estimate(apply(1e4, snr))*FS - 1e4) for _ in range(200)]
    errs.sort()
    print("  snr %2d dB: median err %8.1f Hz   p95 %9.1f Hz" % (snr, errs[100], errs[190]))
