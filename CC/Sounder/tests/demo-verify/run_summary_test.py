# Known answers for run_summary.py's A6 verdict, on the sounder's real line
# formats (a slots-mode run's end-of-run lines). Stdlib only; run from
# tests/demo-verify (ctest does). Each check names the mutation that breaks it.
import contextlib, importlib.util, io, os, sys, tempfile
spec = importlib.util.spec_from_file_location("rs", os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_summary.py"))
rs = importlib.util.module_from_spec(spec); spec.loader.exec_module(rs)
fails = 0
def check(ok, what):
    global fails; print(("PASS " if ok else "FAIL ") + what); fails += (not ok)

GOOD = [
    "57:000100 INFOR: BS: receives only its rx slots (AP-87): TDD_RX_SLOTS active=1 epoch=91566720 stop=0 symbol",
    "57:004619 INFOR: BS 192.168.10.22: RX read check: 507470 stamped reads, 402636 on the count, 104834 after a gap "
    "(0 samples lost in rx slots, 128820113280 the schedule's gaps), 0 out of order, 0 time jumps",
    "57:004621 INFOR: BS 192.168.10.22: AP-87 slot check: 507470 reads, 25763909760 samples, 0 of them outside the rx "
    "slots (0 reads)",
    "57:004972 INFOR: BS 192.168.10.22: RX_HOST_STATUS rxq_ovfl=0 rxq_ovfl_ch0=0 ring_ovfl=0 tdd_drop=0 tdd_drop_ch0=0 "
    "tdd_straddle=0",
    "57:368354 INFOR: UE 192.168.10.21: RX read check: 2411956 stamped reads, 2411956 on the count, 0 after a gap "
    "(0 samples lost in rx slots, 0 the schedule's gaps), 0 out of order, 0 time jumps",
    "57:000200 INFOR: BS 192.168.10.22 link health: [BS 192.168.10.22] 60.0 s: irq 12/s, preflight ok: clean"
    " | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0",
    "3:176656 INFOR: houdiniAcquireAnchor [0]: lock CONFIRMED (resid -1 over 206 frames, confirm 21) -> frame "
    "anchor 140786886, bootstrap period 122879.9613 (-0.0387 samp/frame)",
    "42:198592 INFOR: CNS score 0.995 rot -0.0 deg at frame 553 (512 datagrams, 0 low); P->U 500.0 us, so +180.0 deg "
    "per kHz of uncorrected CFO",
]
def levels(L):
    return [lv for lv, _ in rs.verdict(L)]
def swap(L, old, new):
    return [l.replace(old, new) for l in L]

check("FAIL" not in levels(GOOD) and "WARN" not in levels(GOOD), "a clean slots-mode run passes with no warnings: %s" % rs.verdict(GOOD))
# AP-112, DEMO_VERIFICATION 9.83: clean read checks and counters on a link that
# carried nothing. Each signal on its own, then both gone.
nolock = [l for l in GOOD if "lock CONFIRMED" not in l]
check(any(lv == "FAIL" and "never locked on the beacon" in t for lv, t in rs.verdict(nolock)),
      "a UE that never locked on the beacon fails (mutation: the beacon rule removed)")
# Detections are not a lock: the detector logs a crossing before the SNR floor
# judges it, and a rejected one is a clientSyncBeacon line (the rejection is
# from a real capture, the detection from receiver.cc's format).
hunted = nolock + [
    "2:694464 INFOR: clientSyncBeacon [0]: rejected low-SNR detection (idx 18311, 0.5 dB < 30.0 dB floor), count 1",
    "2:694470 INFOR: syncSearch: detection #1 statistic 0.3121 vs bar 0.3000 (xcorr), idx 18311 in 122880",
    "2:701000 INFOR: clientSyncBeacon [0]: idx 18311 snr 31.2 dB"]
check(any(lv == "FAIL" and "never locked on the beacon" in t for lv, t in rs.verdict(hunted)),
      "detections without a confirmed lock fail (mutation: any detection or clientSyncBeacon line taken as a lock)")
nocns = [l for l in GOOD if "datagrams" not in l]
check(any(lv == "FAIL" and "no constellation record" in t for lv, t in rs.verdict(nocns)),
      "a run whose BS scored no constellation fails (mutation: the constellation rule removed)")
zero = swap(GOOD, "(512 datagrams, 0 low)", "(0 datagrams, 0 low)")
check(any(lv == "FAIL" and "no constellation record" in t for lv, t in rs.verdict(zero)),
      "a CNS summary of zero datagrams is no data (mutation: any summary line taken as data)")
idle = [l for l in GOOD if "lock CONFIRMED" not in l and "datagrams" not in l]
check(sum(lv == "FAIL" for lv in levels(idle)) == 2,
      "the 9.83 shape, clean checks and no data, fails on both counts (mutation: PASS on clean counters alone)")
check("FAIL" in levels(swap(GOOD, "(0 samples lost in rx slots, 128820113280", "(1920 samples lost in rx slots, 128820113280")),
      "samples lost in the rx slots fail (mutation: the lost count not read)")
check("FAIL" in levels([l.replace("0 time jumps", "2 time jumps") if "UE 192" in l else l for l in GOOD]),
      "a time jump fails (mutation: the jump count not read)")
check("FAIL" in levels(swap(GOOD, "0 of them outside", "3 of them outside")), "samples outside the rx slots fail (mutation: the stray count not read)")
check("FAIL" in levels(swap(GOOD, "tdd_drop=0 tdd_drop_ch0", "tdd_drop=5 tdd_drop_ch0")), "a nonzero tdd_drop fails (mutation: tdd_drop not judged)")
check("FAIL" in levels([l for l in GOOD if "UE 192" not in l]),
      "a missing UE read check fails: the run did not close cleanly (mutation: judge only the lines present)")
check("FAIL" in levels([l for l in GOOD if "RX_HOST_STATUS" not in l]),
      "a slots-mode run without RX_HOST_STATUS fails (mutation: host status not required in slots mode)")
allrx = [l for l in GOOD if "rx slots (AP-87)" not in l and "AP-87 slot check" not in l and "RX_HOST_STATUS" not in l]
check("FAIL" not in levels(allrx), "an all-rx run needs no slot check or host status (mutation: slots lines required always)")
check("FAIL" in levels(GOOD + ["57:1 ERROR: terminate called after throwing an instance of 'std::runtime_error'"]),
      "an escaped exception fails (mutation: the error pattern not read)")
app = GOOD + ["57:000300 WARNG: BS 192.168.10.22 link health: [BS 192.168.10.22] 5.0 s: irq 12/s, preflight ok: clean"
              " | app: rx_err +0, rx_short +0, rx_pad +7, tx_short +0, tx_sat +0"]
v = rs.verdict(app)
check("FAIL" not in levels(app) and any(lv == "WARN" and "rx_pad +N" in t for lv, t in v),
      "an app-only health alarm is a warning naming its kind, not a failure (mutation: drop lines reading 'clean |')")
g133 = [l.replace("tdd_drop=0 tdd_drop_ch0=0", "tdd_drop=103400354 tdd_drop_ch0=51700177") for l in GOOD]
check("FAIL" not in levels(g133 + ["57:1 INFOR: stack fpga_version=1.33 fpga_commit=38f662c4"])
      and "FAIL" in levels(g133 + ["57:1 INFOR: stack fpga_version=1.34 fpga_commit=4a19b2f2"]),
      "tdd_drop is judged from fpga 1.34, where the fabric gates; on 1.33 the host drops by design (mutation: "
      "tdd_drop judged on every version, or never)")
lr = GOOD + ["57:%d INFOR: UE TX_HOST_STATUS release_late_ch0=0 release_late_ch1=%d" % (i, min(i, 2)) for i in range(5)]
v = rs.verdict(lr)
check([t for lv, t in v if lv == "WARN"] == ["late releases (host pacer): ch0 0, ch1 2"],
      "late releases are the cumulative counter's peak per channel, not its line count (mutation: count lines)")
# With a readable status beside it (a second radio, or a read that worked once),
# only the explicit rule can fail the unreadable one.
v = rs.verdict(GOOD + ["57:1 WARNG: BS 192.168.10.22: RX_HOST_STATUS unreadable: RX_HOST_STATUS: rxq_ovfl_ch0 is unreadable"])
check(any(lv == "FAIL" and "unreadable" in t for lv, t in v),
      "an unreadable RX_HOST_STATUS fails even beside a readable one (mutation: drop the unreadable rule)")
v = rs.verdict([l for l in GOOD if "RX_HOST_STATUS" not in l]
               + ["57:1 WARNG: BS 192.168.10.22: RX_HOST_STATUS unreadable: RX_HOST_STATUS: rxq_ovfl_ch0 is unreadable"])
check("FAIL" in [lv for lv, _ in v] and not any("counters 0" in t for _, t in v),
      "an unreadable RX_HOST_STATUS alone fails instead of passing with no counters (mutation: match any line naming the key)")
check("FAIL" in levels([l.replace(" tdd_straddle=0", "") for l in GOOD]),
      "an RX_HOST_STATUS without a judged counter fails (mutation: a missing key read as 0)")
edges = GOOD + [  # HoudiniFramer.cc's two slot-edge warnings (S238's and S158's sides, DEMO_VERIFICATION 9.86)
    "0:1 WARNG: BS: the UE burst starts at the pilot slot's edge (pilot_grid_off -31, prefix 32, occurrence 1): the "
    "slot cut takes its head if it is any earlier; recalibrate tx_advance with bs_rx_slots off",
    "0:2 WARNG: BS: the UE burst runs to the pilot slot's far edge (pilot_grid_off 39, postfix 32, occurrence 1): the "
    "slot cut takes its tail if it is any later; raise tx_advance by the settled pilot_grid_off",
    "0:3 WARNG: BS: the UE burst runs to the pilot slot's far edge (pilot_grid_off 40, postfix 32, occurrence 2): the "
    "slot cut takes its tail if it is any later; raise tx_advance by the settled pilot_grid_off"]
w = [t for lv, t in rs.verdict(edges) if lv == "WARN"]
check("FAIL" not in levels(edges) and any("head at the slot cut: 1 lines" in t for t in w) and any("tail at the slot cut: 2 lines" in t for t in w),
      "a burst at either slot edge is a warning to read, each side counted (mutation: the edge lines not read): %s" % w)
rnf = ["Radio open failed (attempt 1 of 2): the radios did not come up: SoapyRPCUnpacker::recv() TIMEOUT; retrying"] + GOOD
old = ["Exception: Radios Not Found. Will attempt a retry... Re-Try Now!"] + GOOD  # the banner before it named its cause
check(all("FAIL" not in levels(x) and any("retried" in t for lv, t in rs.verdict(x) if lv == "WARN") for x in (rnf, old)),
      "a radio open that was retried and recovered is a warning, not a failure, in the banner's new form and its old "
      "(mutation: the open failure as an error, or the old form not read)")
last = GOOD + ["Radio open failed (attempt 2 of 2): the radios did not come up: the TDD ladder's TX_CLEAR_ALL (RFCORE "
               "0x024) did not land; no retry left, the run did not start"]
v = rs.verdict(last)
check(any(lv == "FAIL" and t == "the radio open failed on every attempt: the TDD ladder's TX_CLEAR_ALL (RFCORE 0x024) did not land" for lv, t in v)
      and not any("retried" in t for lv, t in v),
      "the last attempt's line fails naming its cause, and is not counted as a retry (mutation: the no-retry line "
      "read as a retried open): %s" % v)
K = rs.alarm_kinds([
    "57:2 WARNG: BS x link health: [BS x] 5.0 s: irq 1/s, preflight ok: egress.drop_p0 +3; host.rxq_ovfl_ch2 +7"
    " | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0",
    "57:3 WARNG: BS x link health: [BS x] 5.0 s: irq 1/s, preflight ok: egress.drop_p0=255 (saturated: further drops"
    " cannot be counted); config blocks: a -> b | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0"])
check(dict(K) == {"egress.drop_p0 +N": 1, "host.rxq_ovfl_ch2 +N": 1},
      "an egress rise and a per-stream host rise get their kinds, and the retired blind and drift forms none "
      "(mutation: the old 'saturated' or 'config' pattern kept): %s" % dict(K))
lr2 = GOOD + ["57:%d INFOR: UE 192.168.10.21 TX_HOST_STATUS: late_refusals_ch0=%d late_refusals_ch1=0 release_late_ch0=0" % (i, i)
               for i in range(3)] + ["57:9 WARNG: Write rejected: HAS_TIME stamp 123 behind by 50 ms"]
w = [t for lv, t in rs.verdict(lr2) if lv == "WARN"]
check(any("refused by the host" in t and "ch0 2" in t for t in w) and any("Write rejected" in t for t in w)
      and "FAIL" not in levels(lr2),
      "late TX writes the host refused and the plugin's Write rejected lines are warnings for the software lane "
      "(mutation: late_refusals not read)")
K2 = rs.alarm_kinds([
    "57:4 WARNG: UE x link health: [UE x] 5.0 s: irq 1/s, preflight FAIL TX1:late=11: preflight new FAIL TX1:late=11 | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0",
    "57:5 WARNG: UE x link health: [UE x] 5.0 s: irq 1/s, preflight FAIL TX1:late=11: preflight new FAIL TX1:late=11; tx1.late +3 | app: rx_err +0, rx_short +0, rx_pad +0, tx_short +0, tx_sat +0"])
check(K2.get("preflight new FAIL TX1:late=11") == 2 and all(not k.endswith(" ") for k in K2),
      "one alarm counts under one key whatever follows it (mutation: keep the space before a pipe): %s" % dict(K2))
STEER = [
    "  steer.max_push = 2  [default]",
    "37:958398 INFOR: Clock steering [0]: ON: CLOCK_ADJ holdover=1 man_dac=408 rb_dac=408 pll1_locked=0 ref=calibrated",
    "58:429807 INFOR: Clock steering [0]: tracked eps -0.1951 ppm averaged over 20 s -> push -1: landed at 407, offset -1",
    "80:992399 INFOR: Clock steering [0]: tracked eps +0.1597 ppm averaged over 20 s -> push +1: landed at 408, offset +0",
    "32:165260 INFOR: Clock steering [0]: released to the calibrated hold after 2 push(es), offset was 0 counts"]
check(rs.steering_pushes(STEER) == 2,
      "the push count is the push lines only (mutation: the old 'steer.*(push|CLOCK_ADJ)' count, 5 here): %d" % rs.steering_pushes(STEER))
PB = ["37:988647 INFOR: UE pilot burst: scheduled 11 frames up to 490587028 (pad 148)",
      "38:002916 INFOR: UE pilot burst: scheduled 3 frames up to 494273429 (pad 149)"]
check(rs.pilot_frames(PB) == (14, 2), "a schedule line counts its N frames (mutation: count the lines as bursts): %s" % (rs.pilot_frames(PB),))
C31 = ["57:1 INFOR: UE 192.168.10.21 TX_HOST_STATUS: eob_recloses=0 eob_recloses_ch0=0 eob_recloses_ch1=0 offset_us_ch0=-25.8 "
       "offset_samples_ch0=4 offset_implausible_ch0=0 anchor_rejects_ch0=0 rate_ppm_ch0=-0.00 late_refusals_ch0=0 "
       "write_min_margin_us_ch0=29753.6 release_min_margin_us_ch0=-64417.1 release_late_ch0=24691 burst_refusals_ch0=0 "
       "cold_releases_ch0=12710 cold_window_ms_ch0=249.6 ring_bytes_ch0=268435456 ring_slots_ch0=32768"]
w31 = [t for lv, t in rs.verdict(GOOD + C31) if lv == "WARN"]
check(any("late releases" in t and "ch0 24691" in t for t in w31) and not any("12710" in t for t in w31),
      "0.4.0's TX_HOST_STATUS (its key order, the ring keys last): release_late read, the cold and ring keys not "
      "taken for it (mutation: a looser key "
      "pattern): %s" % w31)
# AP-117, W5A1 (DEMO_VERIFICATION 9.84): the X-IF reached no ADC, half the
# records scored low and ant 1's MER read 2.6 dB, and the verdict was PASS.
half = swap(GOOD, "(512 datagrams, 0 low)", "(5120 datagrams, 2560 low)")
check(any(lv == "FAIL" and "2560 of 5120 constellation records low" in t for lv, t in rs.verdict(half)),
      "half the constellation records low fails (mutation: the low fraction not judged)")
check("FAIL" not in levels(swap(GOOD, "(512 datagrams, 0 low)", "(160256 datagrams, 2 low)"))
      and "FAIL" not in levels(swap(GOOD, "(512 datagrams, 0 low)", "(5120 datagrams, 511 low)"))
      and "FAIL" in levels(swap(GOOD, "(512 datagrams, 0 low)", "(5120 datagrams, 512 low)")),
      "the low fraction fails from a tenth: L198's 2 of 160,256 and 511 of 5,120 pass, 512 fails (mutation: any "
      "low record fails, or a floor other than 10 %)")
MODEV = [  # W5S2's lines, the UE's first as there, moved to a channel the BS does not use so its role filter shows
    "9:167121 INFOR: UE 168.6.244.22 mode V: RX ch1: NCO 4380.000 MHz, occupied +-48.600 MHz, zone 2, cal Mode 2, "
    "no channel filter",
    "10:013783 INFOR: BS 168.6.244.21 mode V: RX ch0: NCO 2425.000 MHz, occupied +-23.940 MHz, zone 1, cal Mode 1, "
    "+-24 MHz channel filter ON",
    "10:013783 INFOR: BS 168.6.244.21 mode V: RX ch2: NCO 4380.000 MHz, occupied +-48.600 MHz, zone 2, cal Mode 2, "
    "no channel filter"]
check(rs.bs_bands(GOOD + MODEV + MODEV) == [(0, "sub-6"), (2, "X-IF")],
      "the BS's bands in antenna order, once each, the UE's channel not among them (mutation: the role not read, "
      "or a retried open's lines counted again): %s" % rs.bs_bands(GOOD + MODEV + MODEV))
def mer(a0, a1):
    """mer_sampler lines, one per (sub-6, X-IF) pair (W5A1's format); None leaves the antenna out."""
    return ["18:44:%02d UTC " % i + "; ".join(
        "/ant/%d/cns MER %.1f dB, EVM 1.68 %%, 14064 pts, frame 1675, age 9 ms" % (a, v)
        for a, v in ((0, x0), (1, x1)) if v is not None) for i, (x0, x1) in enumerate(zip(a0, a1))] + [
        "4 samples, 0 stale, failed or empty"]
def mv(m):
    return rs.verdict(GOOD + MODEV, m)
ok = mv(mer([35.5, 35.4, 36.0, 36.2], [30.9, 30.6, 31.9, 31.0]))
check("FAIL" not in [lv for lv, _ in ok] and sum("MER median" in t for lv, t in ok if lv == "PASS") == 2,
      "both antennas over their floors pass, each named (mutation: the MER rule removed): %s" % ok)
w5a1 = mv(mer([35.5, 35.4, 36.0, 36.2], [2.6, 2.7, 2.6, 2.6]))
check(any(lv == "FAIL" and "antenna 1 (X-IF, RX ch2): MER median 2.6" in t for lv, t in w5a1),
      "W5A1's X-IF at 2.6 dB fails, named by band and channel (mutation: one antenna's MER taken for the run's)")
gone = mv(mer([35.5, 35.4, 36.0, 36.2], [None] * 4))
check(any(lv == "FAIL" and "antenna 1" in t and "no MER sample" in t for lv, t in gone),
      "an antenna with no MER sample fails (mutation: judge only the antennas the samples name)")
check("FAIL" in [lv for lv, _ in mv(mer([32.0, 32.4, 32.1, 32.6], [30.0, 30.2, 29.9, 30.1]))]
      and "FAIL" not in [lv for lv, _ in mv(mer([34.0, 34.1, 33.9, 34.2], [28.5, 28.4, 28.9, 28.6]))],
      "each band against its own floor: sub-6 at 32 fails its 33, X-IF at 28.5 passes its 28 (mutation: one floor "
      "for both bands, or the floors swapped)")
check("FAIL" not in [lv for lv, _ in mv(mer([24.5, 35.0, 36.1, 36.0, 35.8], [20.1, 31.0, 31.5, 31.2, 31.4]))],
      "a cold start's low first sample passes on the median (mutation: the minimum judged)")
nb = rs.verdict(GOOD, mer([36.0], [31.0]))
check(any(lv == "WARN" and "no per-antenna MER floor" in t for lv, t in nb) and "FAIL" not in [lv for lv, _ in nb],
      "MER samples with no band line warn that the floors were not judged, and the X-IF's 31 dB fails no sub-6 "
      "floor (mutation: antennas assumed sub-6): %s" % nb)
def stale_mer(n_fresh, n_stale):
    """mer_sampler lines: the sub-6 fresh throughout, the X-IF fresh n_fresh times, then STALE at its last value."""
    return ["18:%02d:00 UTC /ant/0/cns MER 36.0 dB, EVM 1.58 %%, 14064 pts, frame %d, age 9 ms; /ant/1/cns %sMER 31.0 dB, "
            "EVM 2.80 %%, 14040 pts, frame %d, age %d ms" % (i, 750 * i, "STALE " if i >= n_fresh else "",
                                                             750 * min(i, n_fresh - 1), 3 if i < n_fresh else 15000 * i)
            for i in range(n_fresh + n_stale)]
st29 = mv(stale_mer(1, 29))
check(any(lv == "FAIL" and "antenna 1" in t and "29 of 30 MER samples stale" in t for lv, t in st29),
      "an X-IF that stopped after its first sample fails on its stale samples, not passes on its one fresh one "
      "(mutation: STALE samples not read): %s" % [t for lv, t in st29 if "antenna 1" in t])
st_all = mv(stale_mer(0, 4))
check(any(lv == "FAIL" and "4 of 4 MER samples stale" in t for lv, t in st_all)
      and not any("no MER sample" in t for lv, t in st_all),
      "an antenna stale throughout fails as stale, not as never sampled (mutation: stale samples dropped)")
st_end = mv(stale_mer(4, 1))
check(not any(lv == "FAIL" and "antenna 1" in t for lv, t in st_end)
      and any(lv == "PASS" and "antenna 1" in t and "1 more stale" in t for lv, t in st_end),
      "a stale sample at a run's end is counted and named, the fresh ones judged (mutation: any stale sample fails, "
      "or the stale value averaged in): %s" % [t for lv, t in st_end if "antenna 1" in t])
# main: the exit status is the verdict's, read from a run directory's largest log
d = tempfile.mkdtemp(prefix="run_summary_")
open(os.path.join(d, "small_cpu.log"), "w").write("x\n")
open(os.path.join(d, "VL1_051734.log"), "w").write("\n".join(GOOD) + "\n" + "57:1 INFOR: filler\n" * 50)
out = io.StringIO()
with contextlib.redirect_stdout(out):
    rc = rs.main(d)
check(rc == 0 and "VERDICT: PASS" in out.getvalue(), "main reads the run directory's sounder log and returns 0 on a pass (mutation: read the first log)")
open(os.path.join(d, "VL1_051734.log"), "a").write(GOOD[1].replace("0 time jumps", "1 time jumps") + "\n")
with contextlib.redirect_stdout(io.StringIO()):
    rc = rs.main(d)
check(rc == 1, "main returns 1 on a FAIL (mutation: always 0)")
open(os.path.join(d, "VL1_051734.log"), "w").write("\n".join(GOOD + MODEV) + "\n")
open(os.path.join(d, "mer.txt"), "w").write("\n".join(mer([35.5, 35.4, 36.0], [2.6, 2.7, 2.6])) + "\n")
out = io.StringIO()
with contextlib.redirect_stdout(out):
    rc = rs.main(d)
check(rc == 1 and "antenna 1 (X-IF" in out.getvalue(),
      "main judges the run directory's mer.txt (mutation: main passes no MER samples to the verdict)")
import shutil; shutil.rmtree(d)
print("%d failure(s)" % fails); sys.exit(1 if fails else 0)
