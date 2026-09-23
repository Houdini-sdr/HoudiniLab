# Known answers for fstage_report.py's in-band estimators and MER, on a synthetic R2 U slot.
import importlib.util, math, os, sys, tempfile
import numpy as np
sys.argv = ["x"]
spec = importlib.util.spec_from_file_location("fr", os.path.join(os.path.dirname(__file__), "fstage_report.py"))
fr = importlib.util.module_from_spec(spec); spec.loader.exec_module(fr)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)
rng = np.random.default_rng(7)
N, cp, es, nsym = 256, 64, 96, 12
di = np.arange(80, 176, dtype=np.int32)          # R2's 96 data tones
FS = 32767.0
nf_db, sig_db = -80.0, -30.0                     # total noise power, in-band signal power (dBFS)
# |H| carries a known tilt: +0.40 dB from the lowest to the highest data tone,
# and the received slot is shaped by the same H (a flat channel would hide a
# report that reads tilt off the wrong array)
tilt_true = 0.40
hshape = 10 ** ((tilt_true * (di - di.min()) / (di.max() - di.min())) / 20)
slot = np.zeros(4096, complex)
for k in range(nsym):
    Xc = np.zeros(N, complex)
    Xc[di] = hshape * (rng.choice([-1, 1], len(di)) + 1j * rng.choice([-1, 1], len(di))) / math.sqrt(2)
    t = np.fft.ifft(np.fft.ifftshift(Xc))        # DC-centred bins -> time
    t = np.concatenate([t[-cp:], t])
    slot[es + k * (cp + N): es + (k + 1) * (cp + N)] = t
ofdm = slot[es:es + nsym * (cp + N)]
slot *= FS * 10 ** (sig_db / 20) / math.sqrt(np.mean(np.abs(ofdm) ** 2))
slot += FS * 10 ** (nf_db / 20) / math.sqrt(2) * (rng.standard_normal(4096) + 1j * rng.standard_normal(4096))
s16 = np.round(np.stack([slot.real, -slot.imag], 1)).astype(np.int16)   # the dump stores the un-conjugated RX
f = tempfile.NamedTemporaryFile(delete=False, suffix=".bin")
f.write(np.array([N, cp, es, nsym, len(di)], np.int32).tobytes())
H = np.zeros(N, np.complex64); H[di] = hshape
f.write(np.stack([H.real, H.imag], 1).astype(np.float32).tobytes())
f.write(di.tobytes()); f.write(s16.tobytes()); f.close()
u = fr.ul(f.name)
floor_true = nf_db + 10 * math.log10(96 / 256)   # white noise in the 96-of-256 band
check(abs(u["floor_in"] - floor_true) < 1.0, "idle in-band floor %.2f dBFS, truth %.2f (idle segments %d)" % (u["floor_in"], floor_true, u["nidle"]))
check(abs(u["sig_in"] - sig_db) < 0.3, "in-band signal %.2f dBFS, truth %.2f" % (u["sig_in"], sig_db))
snr_true = sig_db - floor_true
check(abs(u["snr_rx"] - snr_true) < 1.0, "SNR_rx %.2f dB, truth %.2f" % (u["snr_rx"], snr_true))
check(abs(u["mer"] - snr_true) < 1.0, "MER %.2f dB matches the in-band SNR %.2f (receive-noise-limited by construction)" % (u["mer"], snr_true))
check(u["nidle"] >= 3, "idle segments found on both sides of the burst")
check(abs(u["tilt"] - tilt_true) < 0.01, "|H| tilt %.3f dB, truth %.2f (lowest to highest tone)" % (u["tilt"], tilt_true))
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
