#!/usr/bin/env python3
"""AP-43's gate: does a TIMED BURST read cost the same fixed ~855 us as a
continuous one? That single number decides the whole item.

WHAT AP-43 IS FOR. The client's beacon read lands at an uncontrolled frame
phase, so only ~1.4% of frames even attempt a correlation, and the FPGA streams
every sample regardless: 495.3 MB/s carried against 16.4 MB/s kept, 97% waste.
A timed burst fixes both -- it puts the read WHERE the beacon is and discards
the rest in the FPGA. Its cost is time management: a late arm MISSES the slot
outright where a continuous stream merely returns stale data. The AP-31 tracker
now predicts the beacon to +-3 samples, so that cost is affordable.

WHAT IS UNKNOWN, AND WHY IT DECIDES EVERYTHING. radioRx measures 855 us fixed
plus 0.0037 us/sample on the continuous path. If a timed burst carries the SAME
fixed cost, then per-frame reads cost 855 us against a 1 ms frame and per-frame
DL reads are unaffordable no matter how little data they carry -- the item is
worth only the fronthaul saving. If the fixed cost is mostly the CONTINUOUS
machinery (drain, backlog, MTU-sized reassembly) and a burst avoids it, the
item is worth a great deal more.

NOTHING ON THE UE HAS EVER CALLED THIS. Every client-side activateRecv() in the
sounder passes no arguments, i.e. flags=0, continuous and untimed
(ClientRadioSet.cc:259,270,274,282). Radio::activateRecv(rxTime, numSamps,
flags) maps flags=2 to SOAPY_SDR_HAS_TIME + SOAPY_SDR_END_BURST and flags=3 to
SOAPY_SDR_WAIT_TRIGGER + SOAPY_SDR_END_BURST; the only call sites that pass
anything are BS calibration. So this probe exercises a driver path the
application lane has never used, and a failure here is a FINDING about the
contract rather than a bug in this script.

  python3 tests/demo-verify/burst_rx_cost_probe.py --ue <ip> --reps 200
"""
import argparse
import math
import statistics
import sys
import time

import numpy as np

import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_HAS_TIME, SOAPY_SDR_END_BURST

# AP-36: the `timeout` device arg is MICROSECONDS and bounds make(), which runs
# the full RFDC bring-up cold and measures 3.34 s on this bench.
RPC_TIMEOUT_US = "30000000"
RATE = 122.88e6
FRAME = 122880
SLOT = 4096


def stats(xs):
    if not xs:
        return None
    xs = sorted(xs)
    return {
        "n": len(xs), "mean": statistics.fmean(xs),
        "med": xs[len(xs) // 2], "min": xs[0], "max": xs[-1],
        "sd": statistics.stdev(xs) if len(xs) > 1 else 0.0,
        "p90": xs[int(0.90 * (len(xs) - 1))],
    }


def show(label, s, nsamps):
    if s is None:
        print("  %-28s NO DATA" % label)
        return
    print("  %-28s n=%4d  mean %8.1f us  med %8.1f  sd %7.1f  min %8.1f  "
          "p90 %8.1f  max %8.1f   (%.4f us/sample)"
          % (label, s["n"], s["mean"], s["med"], s["sd"], s["min"], s["p90"],
             s["max"], s["mean"] / nsamps))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ue", required=True)
    ap.add_argument("--ch", type=int, default=1)
    ap.add_argument("--freq", type=float, default=500e6)
    ap.add_argument("--reps", type=int, default=200)
    ap.add_argument("--nsamps", type=int, default=SLOT,
                    help="samples per read (default one slot)")
    ap.add_argument("--no-deactivate", action="store_true",
                    help="do NOT deactivate between bursts. Correct on a build "
                         "with the END_BURST re-arm fix, and REQUIRED to measure "
                         "the real cycle there.")
    ap.add_argument("--rearm-drain-ms", type=int, default=-1,
                    help="host plugin rx_rearm_drain_ms for leg B's stream "
                         "(0-50). -1 leaves it unset, i.e. the driver default.")
    ap.add_argument("--lead-frames", type=int, default=4,
                    help="how far ahead of now to arm each burst")
    a = ap.parse_args()

    dev = SoapySDR.Device(dict(driver="houdinisdr",
                               remote="tcp://%s:55132" % a.ue,
                               timeout=RPC_TIMEOUT_US))
    hi = dict(dev.getHardwareInfo())
    print("UE %s label=%s ip=%s" % (a.ue, hi.get("label", "?"),
                                    hi.get("ip_address", "")))
    dev.setSampleRate(SOAPY_SDR_RX, a.ch, RATE)
    dev.setFrequency(SOAPY_SDR_RX, a.ch, a.freq)
    buf = np.zeros(2 * max(a.nsamps, 16384), dtype=np.int16)
    print("reads of %d samples, %d reps each\n" % (a.nsamps, a.reps))

    # ---------------- leg A: the continuous path we ship today -------------
    # Drain first, exactly as recvHoudini does, so the timed region is the read
    # and not the backlog. The drain is timed SEPARATELY: the 855 us figure this
    # probe exists to compare against was measured on radioRx as a whole, and
    # attributing it wrongly is how AP-43 gets decided on the wrong number.
    rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [a.ch],
                          dict(local_port=str(10001 + a.ch), rx_gap_break="1"))
    dev.activateStream(rxs)
    time.sleep(0.2)
    drain_us, cont_us, cont_got = [], [], []
    cont_bad, cont_short = 0, 0
    for _ in range(a.reps):
        t0 = time.perf_counter()
        while True:
            dr = dev.readStream(rxs, [buf], 16384, timeoutUs=0)
            if dr.ret <= 0:
                break
        t1 = time.perf_counter()
        sr = dev.readStream(rxs, [buf], a.nsamps, timeoutUs=1000000)
        t2 = time.perf_counter()
        # A SHORT READ IS NORMAL HERE, not a failure. readStream truncates on
        # this platform as a matter of course (ret=2032 against a 12288 request,
        # observed live and counted by clock_drift_probe.py). Only ret <= 0 is a
        # failed read. Counting truncation as failure would have emptied both
        # legs and, in leg B, printed "the driver does not accept a timed RX
        # burst" -- blaming the driver contract for this script's own policy.
        if sr.ret <= 0:
            cont_bad += 1
            continue
        if sr.ret != a.nsamps:
            cont_short += 1
        drain_us.append((t1 - t0) * 1e6)
        cont_us.append((t2 - t1) * 1e6)
        cont_got.append(sr.ret)
    dev.deactivateStream(rxs)
    dev.closeStream(rxs)

    print("=== leg A: continuous stream (what the client does today) ===")
    show("drain only", stats(drain_us), a.nsamps)
    show("read after drain", stats(cont_us), a.nsamps)
    show("drain + read", stats([d + c for d, c in zip(drain_us, cont_us)]), a.nsamps)
    if cont_bad:
        print("  %d failed read(s) (ret <= 0) excluded" % cont_bad)
    if cont_short:
        print("  %d short read(s) INCLUDED, mean %.0f of %d samples requested "
              "-- truncation is normal here and is not a failure"
              % (cont_short, sum(cont_got) / len(cont_got), a.nsamps))
    print()

    # ---------------- leg B: timed burst, arm per read ---------------------
    # Radio::activateRecv(rxTime, numSamps, 2) == HAS_TIME | END_BURST. Arm,
    # read, let the burst end itself. The arm is timed separately from the read
    # because they are different costs with different fixes: an expensive ARM is
    # an RPC round trip per frame, an expensive READ is the data path.
    # rx_rearm_drain_ms is a host-plugin seam the software lane exposed so this
    # drain can be SIZED from a measurement instead of left at a conservative
    # bound. Unset leaves the driver default.
    sargs = dict(local_port=str(10001 + a.ch), rx_gap_break="1")
    if a.rearm_drain_ms >= 0:
        sargs["rx_rearm_drain_ms"] = str(a.rearm_drain_ms)
    rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [a.ch], sargs)
    arm_us, brd_us, brd_got, cycle_us = [], [], [], []
    seam_err = []   # returned first-sample tick minus armed tick
    late, arm_fail, brd_short = 0, 0, 0
    for _ in range(a.reps):
        # THE FULL CYCLE IS THE NUMBER THAT MATTERS. This probe used to time
        # activateStream and readStream between their own clock pairs and quote
        # the ARM as "the cost". deactivateStream sat outside both, and the
        # driver gives it a 50 ms drain -- so the workaround path measured at
        # 1.75 ms actually cost a consumer about 52 ms per burst.
        cycle_t0 = time.perf_counter()
        try:
            now_ns = dev.getHardwareTime()
        except Exception as e:
            print("  getHardwareTime failed: %s" % e)
            break
        # THE START TIME MUST BE A WHOLE MILLISECOND. The driver rejects an
        # off-boundary arm outright rather than snapping it: "start N ns must be
        # a whole millisecond (a multiple of 1000000 ns; or, with a TDD schedule
        # engaged, on the 3125 ns TDD window grid); off-boundary is rejected,
        # not snapped". Measured 2026-09-02: 120 of 120 arms refused because
        # this line added a frame offset to an ARBITRARY `now`, and the probe's
        # own verdict block then reported "the driver does not accept a timed RX
        # burst on this build" -- filing a client-side arithmetic bug as a
        # driver-contract finding, which is exactly the mistake its docstring
        # warns the reader about.
        #
        # The frame is 122880 samples at 122.88 MSPS = exactly 1 ms, so whole
        # millisecond alignment IS frame alignment here; snap UP so the target
        # is never already in the past.
        target = now_ns + a.lead_frames * (FRAME / RATE) * 1e9
        when = int((int(target) // 1000000 + 1) * 1000000)
        t0 = time.perf_counter()
        r = dev.activateStream(rxs, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                               when, a.nsamps)
        t1 = time.perf_counter()
        # RECORD THE ARM COST UNCONDITIONALLY. The arm is complete at t1
        # whatever the read then does, and appending it only on a successful
        # read measures the subset of arms whose reads happened to work -- a
        # selection bias on the single number this probe exists to produce, and
        # it mattered here because half the reads were failing for an unrelated
        # reason (see the deactivate note below).
        if r != 0:
            arm_fail += 1
            continue
        arm_us.append((t1 - t0) * 1e6)
        sr = dev.readStream(rxs, [buf], a.nsamps,
                            timeoutUs=int(2e6 + a.lead_frames * 1e3))
        t2 = time.perf_counter()
        # DEACTIVATE AFTER EVERY BURST, not only after a failed one. An
        # END_BURST read that SUCCEEDS also leaves the stream needing a fresh
        # activate, and without this every second arm landed on a stream that
        # was not ready: measured 2026-09-02, EXACTLY half the bursts returned
        # nothing in all six configurations tried (60/120, 40/80, 40/80, 40/80,
        # 50/100, 50/100). "Exactly half, every time, independent of the lead"
        # is the shape of a state-machine bug in the caller, not of a late arm,
        # and it was reported as the latter.
        # ...and on a build WITH the re-arm fix the deactivate is unnecessary
        # AND ruinous to measure with: it carries a 50 ms driver drain that
        # measured 66 ms of a 74.5 ms cycle, so the workaround dominated the
        # quantity being measured. --no-deactivate is required there.
        if not a.no_deactivate:
            try:
                dev.deactivateStream(rxs)
            except Exception:
                pass
        # A MISSED slot is ret <= 0. A truncated read is ordinary here and its
        # timing still measures what this probe is for.
        if sr.ret <= 0:
            late += 1
            continue
        if sr.ret != a.nsamps:
            brd_short += 1
        brd_us.append((t2 - t1) * 1e6)
        brd_got.append(sr.ret)
        cycle_us.append((time.perf_counter() - cycle_t0) * 1e6)
        # THE SEAM CHECK. The joinless re-arm accepts one residual by design: a
        # straggler from the PREVIOUS burst arriving at the head of the next.
        # It is directly visible -- compare the time the burst was ARMED for
        # against the time of the first sample actually returned. Equal means
        # the read starts where it was told to. EARLIER means samples from the
        # previous burst led the buffer, which is the straggler.
        if sr.flags & SOAPY_SDR_HAS_TIME:
            armed_tick = int(round(when * RATE / 1e9))
            got_tick = int(round(sr.timeNs * RATE / 1e9))
            seam_err.append(got_tick - armed_tick)
    try:
        dev.deactivateStream(rxs)
    except Exception:
        pass
    dev.closeStream(rxs)

    print("=== leg B: timed burst (HAS_TIME | END_BURST), armed per read ===")
    show("arm (activateStream)", stats(arm_us), a.nsamps)
    show("read after arm", stats(brd_us), a.nsamps)
    show("arm + read", stats([x + y for x, y in zip(arm_us, brd_us)]), a.nsamps)
    if arm_fail:
        print("  %d arm(s) REFUSED by the driver (activateStream returned "
              "non-zero). Read the driver's own error line above before "
              "concluding anything: an off-boundary start time is refused "
              "outright, and that is a bug in this script's arithmetic rather "
              "than a statement about the contract." % arm_fail)
    if late:
        print("  %d burst(s) returned NOTHING -- a LATE ARM misses the slot "
              "outright, which is the cost AP-43 names" % late)
    if brd_short:
        print("  %d burst(s) truncated, mean %.0f of %d -- normal, included"
              % (brd_short, sum(brd_got) / len(brd_got), a.nsamps))
    print()

    # ---------------- the verdict AP-43 asked for --------------------------
    cb = stats(brd_us)
    ta = stats([d + c for d, c in zip(drain_us, cont_us)])
    tb = stats([x + y for x, y in zip(arm_us, brd_us)]) if brd_us else None
    print("=== AP-43 verdict ===")
    if not ta:
        print("  NO CONTINUOUS DATA: every leg-A read failed (ret <= 0). There")
        print("  is nothing to compare against, so this is a bench problem")
        print("  rather than an AP-43 result. Check the link before rerunning.")
        return 1
    if not cb or not tb:
        print("  NO BURST DATA. Before concluding anything about the driver,")
        print("  check WHY the arms failed -- the driver prints a specific")
        print("  reason and an off-boundary start time is a bug in this script,")
        print("  not a contract limitation. Only a refusal the driver attributes")
        print("  to the BURST MODE ITSELF makes this a driver ask.")
        return 1
    cy = stats(cycle_us)
    frame_us = FRAME / RATE * 1e6
    if cy:
        rd = stats(brd_us)["mean"] if brd_us else 0.0
        am = stats(arm_us)["mean"] if arm_us else 0.0
        print("  burst FULL CYCLE           %8.1f us  (%.1f frames)  <-- quote THIS"
              % (cy["mean"], cy["mean"] / frame_us))
        print("     of which arm %.0f us, read %.0f us, teardown %.0f us"
              % (am, rd, cy["mean"] - am - rd))
        print("     at a 260 ms beacon cadence that is %.2f%% overhead"
              % (cy["mean"] / 260000.0 * 100))
        print()
    if seam_err:
        arr = np.asarray(seam_err, dtype=np.int64)
        exact = int((arr == 0).sum())
        early = int((arr < 0).sum())
        print("  SEAM: %d of %d bursts started EXACTLY where armed; %d early "
              "(straggler), %d late" % (exact, len(arr), early,
                                        int((arr > 0).sum())))
        if early:
            print("        earliest lead-in %d samples (%.1f us of the previous "
                  "burst at the head of this one)"
                  % (int(arr.min()), abs(int(arr.min())) / RATE * 1e6))
        print()
    print("  continuous, drain + read   %8.1f us" % ta["mean"])
    print("  burst, arm + read          %8.1f us  (a COMPONENT, not the cost)"
          % tb["mean"])
    print("  ratio                      %8.2fx" % (tb["mean"] / ta["mean"]))
    print()
    print("  Against the 1 ms frame: continuous %.0f%% of a frame, burst %.0f%%."
          % (ta["mean"] / 1000.0 * 100, tb["mean"] / 1000.0 * 100))
    if tb["mean"] < 1000.0:
        print("  A per-frame timed read FITS inside a frame. Per-frame DL reads")
        print("  are affordable and AP-43 is worth more than its fronthaul saving.")
    else:
        print("  A per-frame timed read does NOT fit inside a frame. AP-43 is")
        print("  then worth only the fronthaul saving (97%% of 495.3 MB/s), and")
        print("  per-frame DL reads need a different mechanism.")
    print()
    print("  Reminder: one run is not a behaviour. Repeat this leg at least")
    print("  twice before either conclusion goes in the ledger.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
