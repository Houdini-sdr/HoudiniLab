# Known answers for fstage_report.py's in-band estimators and MER, on a synthetic R2 U slot,
# its re-sync window figures, and its per-stage grouping.
import contextlib, importlib.util, io, math, os, shutil, sys, tempfile
import numpy as np
spec = importlib.util.spec_from_file_location("fr", os.path.join(os.path.dirname(os.path.abspath(__file__)), "fstage_report.py"))
fr = importlib.util.module_from_spec(spec)
imported = io.StringIO()
with contextlib.redirect_stdout(imported):
    spec.loader.exec_module(fr)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)
# Fails under: dropping the __main__ guard (the import then runs the report and
# rewrites /tmp/fstage_report_last.json, the last real report, with {}).
check(imported.getvalue() == "", "importing the module runs no report (printed %d chars)" % len(imported.getvalue()))
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
# A slot cut short inside its last symbol (R3's symbols run to the slot's end):
# the report must still read it, from the whole symbols it has.
cut = tempfile.NamedTemporaryFile(delete=False, suffix=".bin")
cut.write(open(f.name, "rb").read()[:-4 * (4096 - (es + nsym * (cp + N) - 100))])
cut.close()
uc = fr.ul(cut.name)
check(abs(uc["sig_in"] - sig_db) < 0.3 and uc["nsym"] == nsym - 1,
      "a slot cut inside its last symbol reads from its %d whole symbols (signal %.2f dBFS)" % (uc["nsym"], uc["sig_in"]))

# ---- the DL figures from re-sync windows -----------------------------------
def write_iq(path, x):
    np.stack([x.real, x.imag], 1).round().astype(np.int16).tofile(path)
def make_run(d, k, dump=None):
    """A run dir with a beacon RAM (the beacon between zero pads) and one 4096-sample
    re-sync window: white noise of 10 rms per axis and the beacon at k."""
    os.makedirs(os.path.join(d, "resync"))
    write_iq(os.path.join(d, "beacon_ram.bin"), np.concatenate([np.zeros(100), beacon, np.zeros(100)]))
    x = 10.0 * (rng.standard_normal(4096) + 1j * rng.standard_normal(4096))
    x[k:k + len(beacon)] += beacon
    write_iq(os.path.join(d, "resync", "resyncwin_0000.bin"), x)
    if dump:
        shutil.copy(dump, os.path.join(d, "cns_dump.bin"))
    return d
beacon = 8000.0 * np.exp(2j * np.pi * rng.random(600))
band = fr.chan_band(N, di)
noise_in = 10 * math.log10(200.0 * 96 / 256 / FS ** 2)   # 2 x 10^2 per sample, 96 of 256 bins
root = tempfile.mkdtemp(prefix="fstage_test_")
# A beacon inside the first 320 samples leaves no whole 256-sample segment
# before it: the floor must come from after it only.
dk = fr.dl(make_run(os.path.join(root, "r2_000001"), 200), band)
# Fails under: rig_dumps.segments allowing one segment at the region start when the
# region is shorter than M (range(a, max(a, b - M) + 1, hop), the old max(0, ...)).
check(abs(dk["floor_in"] - noise_in) < 1.0, "beacon at 200: DL in-band floor %.2f dBFS, the noise's %.2f" % (dk["floor_in"], noise_in))
dk = fr.dl(make_run(os.path.join(root, "r2_000002"), 1500), band)
check(abs(dk["floor_in"] - noise_in) < 1.0, "beacon at 1500: DL in-band floor %.2f dBFS, the noise's %.2f" % (dk["floor_in"], noise_in))
# The in-band floor needs the run's OWN uplink dump for its band: a run without
# one reports none, even straight after a run that had one.
ra = fr.run_row(make_run(os.path.join(root, "r2_000003"), 1500, dump=f.name))
rb = fr.run_row(make_run(os.path.join(root, "r2_000004"), 1500))
# Fails under: run_row caching the band across runs (the old module-level DL_BAND).
check("dl_floor_in" in ra and "dl_floor_in" not in rb, "the DL in-band floor is per run (with a dump %s, without %s)"
      % ("dl_floor_in" in ra, "dl_floor_in" in rb))

# ---- per-stage means never mix rungs ---------------------------------------
def row(run, mer, n):
    return {"run": run, "ul0_mer": mer, "ul0_N": n}
stages = [("F0", [row("r2_000001", 45.0, 256), row("r2_000002", 45.5, 256), row("r3_000003", 30.0, 4096)]),
          ("F1", [row("r2_000004", 44.0, 256), row("r3_000005", 29.0, 4096)])]
ss = fr.stage_stats(stages)
r2, r3 = ss["r2"]["ul0_mer"], ss["r3"]["ul0_mer"]
# Fails under: stage_stats taking every run of the stage for each tag's values
# (dropping the tag_of filter: the R3 run's 30 dB then enters the R2 mean).
check(abs(r2[0][0] - 45.25) < 1e-9 and abs(r2[0][1] - 0.5) < 1e-9 and abs(r2[1][2] - (44.0 - 45.25)) < 1e-9,
      "R2 runs are averaged on their own (mean %.2f, range %.2f, change %+.2f)" % (r2[0][0], r2[0][1], r2[1][2]))
check(abs(r3[0][0] - 30.0) < 1e-9 and abs(r3[1][2] + 1.0) < 1e-9, "R3 runs are averaged on their own (change %+.2f)" % r3[1][2])
try:
    fr.stage_stats([("F0", [row("r2_000001", 45.0, 256), row("r2_000002", 30.0, 4096)])])
    refused = False
except ValueError:
    refused = True
# Fails under: removing the numerology check in stage_stats.
check(refused, "one tag reused for two numerologies is refused")
shutil.rmtree(root)
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
