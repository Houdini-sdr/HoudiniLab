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
    drain_us, cont_us, cont_bad = [], [], 0
    for _ in range(a.reps):
        t0 = time.perf_counter()
        while True:
            dr = dev.readStream(rxs, [buf], 16384, timeoutUs=0)
            if dr.ret <= 0:
                break
        t1 = time.perf_counter()
        sr = dev.readStream(rxs, [buf], a.nsamps, timeoutUs=1000000)
        t2 = time.perf_counter()
        if sr.ret != a.nsamps:
            cont_bad += 1
            continue
        drain_us.append((t1 - t0) * 1e6)
        cont_us.append((t2 - t1) * 1e6)
    dev.deactivateStream(rxs)
    dev.closeStream(rxs)

    print("=== leg A: continuous stream (what the client does today) ===")
    show("drain only", stats(drain_us), a.nsamps)
    show("read after drain", stats(cont_us), a.nsamps)
    show("drain + read", stats([d + c for d, c in zip(drain_us, cont_us)]), a.nsamps)
    if cont_bad:
        print("  %d short/failed read(s) excluded" % cont_bad)
    print()

    # ---------------- leg B: timed burst, arm per read ---------------------
    # Radio::activateRecv(rxTime, numSamps, 2) == HAS_TIME | END_BURST. Arm,
    # read, let the burst end itself. The arm is timed separately from the read
    # because they are different costs with different fixes: an expensive ARM is
    # an RPC round trip per frame, an expensive READ is the data path.
    rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [a.ch],
                          dict(local_port=str(10001 + a.ch), rx_gap_break="1"))
    arm_us, brd_us, late, arm_fail = [], [], 0, 0
    for _ in range(a.reps):
        try:
            now_ns = dev.getHardwareTime()
        except Exception as e:
            print("  getHardwareTime failed: %s" % e)
            break
        # Arm a whole number of frames ahead so the burst lands on a frame
        # boundary; the exact phase does not matter for a COST measurement.
        when = int(now_ns + a.lead_frames * (FRAME / RATE) * 1e9)
        t0 = time.perf_counter()
        r = dev.activateStream(rxs, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                               when, a.nsamps)
        t1 = time.perf_counter()
        if r != 0:
            arm_fail += 1
            continue
        sr = dev.readStream(rxs, [buf], a.nsamps,
                            timeoutUs=int(2e6 + a.lead_frames * 1e3))
        t2 = time.perf_counter()
        if sr.ret != a.nsamps:
            late += 1
            try:
                dev.deactivateStream(rxs)
            except Exception:
                pass
            continue
        arm_us.append((t1 - t0) * 1e6)
        brd_us.append((t2 - t1) * 1e6)
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
        print("  %d arm(s) refused by the driver" % arm_fail)
    if late:
        print("  %d burst(s) returned short -- a LATE ARM misses the slot "
              "outright, which is the cost AP-43 names" % late)
    print()

    # ---------------- the verdict AP-43 asked for --------------------------
    ca, cb = stats(cont_us), stats(brd_us)
    ta = stats([d + c for d, c in zip(drain_us, cont_us)])
    tb = stats([x + y for x, y in zip(arm_us, brd_us)]) if brd_us else None
    print("=== AP-43 verdict ===")
    if not cb or not tb:
        print("  NO BURST DATA. If every arm was refused, the driver does not")
        print("  accept a timed RX burst on this build and THAT is the finding:")
        print("  the item becomes a driver ask, not an application change.")
        return 1
    print("  continuous, drain + read   %8.1f us" % ta["mean"])
    print("  burst, arm + read          %8.1f us" % tb["mean"])
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
