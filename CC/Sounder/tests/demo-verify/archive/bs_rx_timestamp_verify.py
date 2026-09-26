#!/usr/bin/env python3
"""BS RX timestamp/slot verification (DEMO_VERIFICATION.md section 3/4).

Arms the framer exactly as the slot-granular sounder does (all-'2' ring with
'6' on slot 0, symbol_ticks = 4096, spf = 30, single-burst strobe), then
verifies from the RX stream alone, WITHOUT looking at data content:

  V1  every read carries HAS_TIME;
  V2  stream continuity is tick-exact (this_stamp == prev_stamp + prev_ret,
      1 sample = 1 tick at 122.88 MSPS);
  V3  each stamp maps to (frame, slot, sample) against the TDD_ARM epoch:
      delta = ticks - epoch; frame = delta // 122880;
      slot = (delta % 122880) // 4096; samp = delta % 4096;
  V4  frame-sized reads (122880 samples) advance frame by exactly +1 with
      (slot, samp) unchanged, i.e. the app-layer symbol mapping is stable;
  V5  total delivered samples across N frame-reads == N * 122880 (no loss,
      no padding).

Exit hygiene: full teardown ladder + strobe off + streams closed.
"""
import argparse
import json
import sys
import time

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_TX

# SoapyRemote's `timeout` device arg is MICROSECONDS and it bounds the make()
# RPC, not the stream. Measured on this bench: a COLD make (the server holds no
# live device instance, so construction runs the full RFDC bring-up) takes
# 3.34 s, a WARM one 0.34 s, so the long-standing 1000000 (= 1 s) sat INSIDE the
# normal spread. A `SoapyRPCUnpacker::recv() TIMEOUT` on make is that, NOT an
# unresponsive server: three were misread as a session wedge in one session
# before it was measured. readStream's timeoutUs is a different thing and stays.
RPC_TIMEOUT_US = "30000000"

RATE = 122.88e6
TICKS_PER_FRAME = 122880
TICKS_PER_SLOT = 4096
SPF = 30


def ns_to_ticks(t_ns):
    return int(round(t_ns * RATE / 1e9))


def fss(ticks, epoch):
    d = ticks - epoch
    return d // TICKS_PER_FRAME, (d % TICKS_PER_FRAME) // TICKS_PER_SLOT, \
        d % TICKS_PER_SLOT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--out", default="bs_rx_timestamp_verify.jsonl")
    args = ap.parse_args()

    dev = SoapySDR.Device(dict(driver="houdinisdr",
                               remote="tcp://%s:55132" % args.ip,
                               timeout=RPC_TIMEOUT_US))
    rec, fails = [], []
    rxs = txs = None

    def fail(tag, msg):
        fails.append(tag)
        print("  FAIL [%s] %s" % (tag, msg))

    try:
        # preamble, sounder order
        dev.setSampleRate(SOAPY_SDR_RX, 1, RATE)
        dev.setSampleRate(SOAPY_SDR_TX, 1, RATE)
        dev.setFrequency(SOAPY_SDR_RX, 1, 500e6)
        dev.setFrequency(SOAPY_SDR_TX, 1, 500e6)
        rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [1],
                              dict(local_port="10002", rx_gap_break="1"))
        txs = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [1],
                              dict(tx_mode="replay"))
        dev.writeSetting("TDD_CMD", "abort")
        try:
            dev.writeSetting("TDD_REPLAY_STROBE", "ch1:off")
        except Exception as e:  # noqa: BLE001
            print("  strobe off threw (ok): %s" % e)
        iq = np.zeros(8192, dtype=np.int16)  # RAM content irrelevant here
        dev.writeStream(txs, [iq], 4096)
        sched = "6" + "2" * (SPF - 1)
        dev.writeSetting("TDD_SCHED", sched)
        dev.writeSetting("TDD_REPLAY_STROBE", "ch1:len=1856,loops=1,offs=384")
        dev.writeSetting("TDD_ARM",
                         "symbol_ticks=%d,symbols_per_frame=%d,margin=36864000"
                         % (TICKS_PER_SLOT, SPF))
        rb = dict(kv.split("=", 1)
                  for kv in dev.readSetting("TDD_ARM").split() if "=" in kv)
        epoch = int(rb["epoch"])
        print("armed: sched=%s epoch=%d accepted=%s"
              % (sched, epoch, rb.get("accepted")))
        if rb.get("accepted") != "1":
            fail("ARM", "not accepted")
            return 4

        dev.activateStream(rxs)
        buf = np.zeros(2 * TICKS_PER_FRAME, dtype=np.int16)

        # --- warm-up small reads: V1..V3 fine-grained
        prev_end = None
        print("first reads (frame/slot/samp derived from stamps):")
        for i in range(8):
            sr = dev.readStream(rxs, [buf], 8192, timeoutUs=1000000)
            if sr.ret <= 0:
                fail("V1", "read %d ret=%d" % (i, sr.ret))
                return 5
            if not (sr.flags & 0x4):
                fail("V1", "read %d without HAS_TIME" % i)
            tk = ns_to_ticks(sr.timeNs)
            f, s, m = fss(tk, epoch)
            gap = 0 if prev_end is None else tk - prev_end
            print("  read[%d] ret=%-5d frame=%-4d slot=%-2d samp=%-4d gap=%d"
                  % (i, sr.ret, f, s, m, gap))
            if prev_end is not None and gap != 0:
                fail("V2", "read %d gap=%d ticks" % (i, gap))
            prev_end = tk + sr.ret
            rec.append(dict(kind="small", i=i, ret=int(sr.ret),
                            ticks=tk, frame=int(f), slot=int(s), samp=int(m)))

        # --- align to a frame boundary, then frame-sized reads: V4/V5
        tk = prev_end
        f, s, m = fss(tk, epoch)
        skip = (TICKS_PER_FRAME - ((tk - epoch) % TICKS_PER_FRAME)) \
            % TICKS_PER_FRAME
        while skip > 0:
            n = min(skip, TICKS_PER_FRAME)
            sr = dev.readStream(rxs, [buf], int(n), timeoutUs=1000000)
            if sr.ret <= 0:
                fail("ALIGN", "ret=%d" % sr.ret)
                return 6
            skip -= sr.ret
        base = None
        total = 0
        bad_frames = 0
        t0 = time.time()
        for k in range(args.frames):
            got = 0
            first_tk = None
            while got < TICKS_PER_FRAME:
                sr = dev.readStream(rxs, [buf], TICKS_PER_FRAME - got,
                                    timeoutUs=1000000)
                if sr.ret <= 0:
                    fail("V5", "frame %d ret=%d after %d" % (k, sr.ret, got))
                    return 7
                if first_tk is None:
                    first_tk = ns_to_ticks(sr.timeNs)
                got += sr.ret
            total += got
            f, s, m = fss(first_tk, epoch)
            if base is None:
                base = (f, s, m)
                print("frame-read base: frame=%d slot=%d samp=%d" % (f, s, m))
                if (s, m) != (0, 0):
                    fail("V4", "alignment landed at slot=%d samp=%d" % (s, m))
            else:
                expect_f = base[0] + k
                if (f, s, m) != (expect_f, base[1], base[2]):
                    bad_frames += 1
                    if bad_frames <= 5:
                        fail("V4", "frame-read %d at (f=%d,s=%d,m=%d), "
                             "expected (f=%d,s=%d,m=%d)"
                             % (k, f, s, m, expect_f, base[1], base[2]))
            rec.append(dict(kind="frame", k=k, first_ticks=first_tk,
                            frame=int(f), slot=int(s), samp=int(m),
                            got=int(got)))
        dt = time.time() - t0
        print("frame reads: %d frames, %d samples total (want %d), "
              "%.1f frames/s wall, bad_frames=%d"
              % (args.frames, total, args.frames * TICKS_PER_FRAME,
                 args.frames / dt, bad_frames))
        if total != args.frames * TICKS_PER_FRAME:
            fail("V5", "sample count mismatch")
        if bad_frames:
            fail("V4", "%d/%d frame reads mis-mapped" % (bad_frames,
                                                         args.frames))
        print("VERDICT: %s" % ("PASS" if not fails else
                               "FAIL(%s)" % ",".join(sorted(set(fails)))))
    finally:
        try:
            dev.writeSetting("TDD_CMD", "abort")
            dev.writeRegister("RFCORE", 0x24, 1)
            dev.writeRegister("RFCORE", 0x24, 0)
            dev.writeSetting("TDD_CMD", "gate_release")
            dev.writeSetting("TDD_REPLAY_STROBE", "ch1:off")
        except Exception as e:  # noqa: BLE001
            print("teardown threw: %s" % e)
        for h in (rxs, txs):
            if h is not None:
                try:
                    dev.deactivateStream(h)
                except Exception:  # noqa: BLE001
                    pass
                try:
                    dev.closeStream(h)
                except Exception as e:  # noqa: BLE001
                    print("closeStream threw: %s" % e)
        with open(args.out, "w") as fjson:
            for r in rec:
                fjson.write(json.dumps(r, sort_keys=True) + "\n")
        print("wrote %d records to %s" % (len(rec), args.out))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
