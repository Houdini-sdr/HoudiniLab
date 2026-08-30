#!/usr/bin/env python3
"""Phase 3/4 experiments on the BS TDD framer, using the sounder's exact
arming values (BaseRadioSet::armHoudiniTdd) but a hygiene-correct teardown.

Experiment A (arming contract, DEMO_VERIFICATION.md section 3):
  A1: from idle, replay the sounder's own sequence verbatim:
      abort -> TDD_SCHED "62" -> strobe on -> TDD_ARM.  Record accept/refuse.
  A2: while running, abort, then re-arm WITHOUT gate_release (the sounder's
      retry pattern).  Record accept/refuse.  Then full ladder and re-arm.

Experiment B (beacon liveness triple, section 4):
  With the framer running, poll TDD_STAT + TX_BANK_STATUS for --secs seconds:
  expect pos_frame advancing ~1000/s, acked tracking pos_frame 1:1,
  smiss == 0, edge_late == 0.  RAM content is irrelevant to the counters.

No streams are opened; settings only.  Exit nonzero if teardown leaves the
framer non-idle or gates held.
"""
import argparse
import json
import sys
import time

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_TX

SOUNDER_SCHED = "62"
SOUNDER_STROBE_ON = "ch1:len=2048,loops=forever,offs=384"
SOUNDER_ARM = "symbol_ticks=61440,symbols_per_frame=2,margin=36864000"


def stat(dev):
    s = dev.readSetting("TDD_STAT")
    return dict(kv.split("=", 1) for kv in s.split() if "=" in kv)


def bank(dev, ch=1):
    txt = dev.readSetting("TX_BANK_STATUS")
    for part in txt.split(";"):
        if part.startswith("ch%d:" % ch):
            return dict(kv.split("=", 1)
                        for kv in part.split(":", 1)[1].split(","))
    return {}


def try_arm(dev, label):
    try:
        dev.writeSetting("TDD_ARM", SOUNDER_ARM)
        rb = dev.readSetting("TDD_ARM")
        print("  [%s] TDD_ARM accepted, readback: %s" % (label, rb))
        return True, rb
    except Exception as e:  # noqa: BLE001
        rb = "<unreadable>"
        try:
            rb = dev.readSetting("TDD_ARM")
        except Exception:  # noqa: BLE001
            pass
        print("  [%s] TDD_ARM THREW: %s\n        readback: %s" % (label, e, rb))
        return False, str(e)


def full_ladder(dev):
    dev.writeSetting("TDD_CMD", "abort")
    dev.writeRegister("RFCORE", 0x24, 1)
    dev.writeRegister("RFCORE", 0x24, 0)
    dev.writeSetting("TDD_CMD", "gate_release")


def main():
    global SOUNDER_SCHED, SOUNDER_STROBE_ON, SOUNDER_ARM
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--secs", type=int, default=60)
    ap.add_argument("--out", default="tdd_arm_experiment.jsonl")
    ap.add_argument("--preamble", action="store_true",
                    help="full sounder-fidelity preamble: rates + NCO + "
                         "setupStream RX/TX-replay + 4096-sample RAM load "
                         "(what Radio.cc + armHoudiniTdd actually do)")
    ap.add_argument("--sched", default=SOUNDER_SCHED,
                    help="TDD_SCHED nibble string (default: sounder's '62')")
    ap.add_argument("--strobe", default=SOUNDER_STROBE_ON,
                    help="TDD_REPLAY_STROBE on-string")
    ap.add_argument("--symbol-ticks", type=int, default=61440)
    ap.add_argument("--spf", type=int, default=2)
    ap.add_argument("--skip-a2", action="store_true",
                    help="liveness only; skip the abort/re-arm experiments")
    ap.add_argument("--rx-probe", type=int, default=0, metavar="N",
                    help="after arming, activate the RX stream and log N raw "
                         "readStream results (ret, flags, stamp, delta) to "
                         "characterize gated-window delivery; implies "
                         "--preamble semantics for the RX stream")
    args = ap.parse_args()
    SOUNDER_SCHED = args.sched
    SOUNDER_STROBE_ON = args.strobe
    SOUNDER_ARM = ("symbol_ticks=%d,symbols_per_frame=%d,margin=36864000"
                   % (args.symbol_ticks, args.spf))
    rec = []
    dev = SoapySDR.Device(dict(driver="houdinisdr",
                               remote="tcp://%s:55132" % args.ip,
                               timeout="1000000"))
    rc = 0
    rxs = txs = None
    try:
        if args.preamble:
            print("== preamble: rates + NCO + streams + RAM (sounder order) ==")
            dev.setSampleRate(SOAPY_SDR_RX, 1, 122.88e6)
            dev.setSampleRate(SOAPY_SDR_TX, 1, 122.88e6)
            dev.setFrequency(SOAPY_SDR_RX, 1, 500e6)
            dev.setFrequency(SOAPY_SDR_TX, 1, 500e6)
            rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [1],
                                  dict(local_port="10002", rx_gap_break="1"))
            txs = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [1],
                                  dict(tx_mode="replay"))
            # abort + strobe off BEFORE the RAM load (fill refused while
            # strobed), mirroring armHoudiniTdd's order
            dev.writeSetting("TDD_CMD", "abort")
            try:
                dev.writeSetting("TDD_REPLAY_STROBE", "ch1:off")
            except Exception as e:  # noqa: BLE001
                print("  strobe off threw (sounder swallows): %s" % e)
            # 4096-sample load: a small CW tone at 0.6 FS so a later capture
            # can see it (content is irrelevant to the counters)
            n = np.arange(4096, dtype=np.float64)
            tone = np.round(0.6 * 32767 * np.exp(2j * np.pi * 0.01 * n))
            iq = np.zeros(8192, dtype=np.int16)
            iq[0::2] = tone.real.astype(np.int16)
            iq[1::2] = tone.imag.astype(np.int16)
            r = dev.writeStream(txs, [iq], 4096)
            print("  RAM load writeStream ret=%s" % r.ret)

        print("== A1: sounder's verbatim sequence from idle ==")
        print("  pre state: %s" % stat(dev))
        dev.writeSetting("TDD_CMD", "abort")          # sounder line 1
        s = stat(dev)
        print("  after abort-on-idle: gates_held=%s state=%s"
              % (s.get("gates_held"), s.get("state")))
        try:
            dev.writeSetting("TDD_REPLAY_STROBE", "ch1:off")
        except Exception as e:  # noqa: BLE001
            print("  strobe off threw (sounder swallows this): %s" % e)
        dev.writeSetting("TDD_SCHED", SOUNDER_SCHED)
        dev.writeSetting("TDD_REPLAY_STROBE", SOUNDER_STROBE_ON)
        ok_a1, info_a1 = try_arm(dev, "A1")
        rec.append({"exp": "A1", "post_abort_stat": s, "armed": ok_a1,
                    "info": info_a1})

        if not ok_a1:
            print("  A1 refused; running full ladder then re-arm to proceed")
            full_ladder(dev)
            dev.writeSetting("TDD_SCHED", SOUNDER_SCHED)
            dev.writeSetting("TDD_REPLAY_STROBE", SOUNDER_STROBE_ON)
            ok, info = try_arm(dev, "A1-ladder")
            rec.append({"exp": "A1-ladder", "armed": ok, "info": info})
            if not ok:
                print("  cannot arm even after ladder; abort experiment")
                return 4

        print("== B: liveness triple over %ds ==" % args.secs)
        t0 = time.time()
        s0, b0 = stat(dev), bank(dev)
        print("  t=0 pos_frame=%s acked=%s smiss=%s edge_late=%s state=%s"
              % (s0.get("pos_frame"), b0.get("acked"), b0.get("smiss"),
                 s0.get("edge_late"), s0.get("state")))
        rec.append({"exp": "B", "t": 0, "stat": s0, "bank": b0})
        while time.time() - t0 < args.secs:
            time.sleep(10)
            s1, b1 = stat(dev), bank(dev)
            dt = time.time() - t0
            df = int(s1.get("pos_frame", 0)) - int(s0.get("pos_frame", 0))
            da = int(b1.get("acked", 0)) - int(b0.get("acked", 0))
            print("  t=%.0fs frames+%d acked+%d (ratio %.4f) smiss=%s "
                  "edge_late=%s played=%s late=%s gated=%s"
                  % (dt, df, da, (da / df if df else 0.0), b1.get("smiss"),
                     s1.get("edge_late"), b1.get("played"), b1.get("late"),
                     b1.get("gated")))
            rec.append({"exp": "B", "t": round(dt, 1), "stat": s1, "bank": b1})

        if args.rx_probe > 0 and rxs is not None:
            print("== RX probe: %d raw reads under the gated schedule ==" %
                  args.rx_probe)
            act = dev.activateStream(rxs)
            print("  activateStream(rx) ret=%d" % act)
            buf = np.zeros(2 * 8192, dtype=np.int16)
            prev_t = None
            probe = []
            for i in range(args.rx_probe):
                sr = dev.readStream(rxs, [buf], 8192, timeoutUs=1000000)
                d_ns = (sr.timeNs - prev_t) if prev_t is not None else 0
                nz = int(np.count_nonzero(buf[:max(sr.ret, 0) * 2]))
                rms = 0.0
                if sr.ret > 0:
                    w = buf[:sr.ret * 2].astype(np.float64)
                    rms = float(np.sqrt(np.mean(w * w)))
                row = dict(i=i, ret=int(sr.ret), flags=int(sr.flags),
                           timeNs=int(sr.timeNs), delta_ns=int(d_ns),
                           nonzero=nz, rms=round(rms, 1))
                probe.append(row)
                if i < 30 or sr.ret <= 0:
                    print("  read[%02d] ret=%d flags=0x%x t=%d dt=%d rms=%.1f"
                          % (i, sr.ret, sr.flags, sr.timeNs, d_ns, rms))
                if sr.ret > 0:
                    prev_t = sr.timeNs + int(round(sr.ret * 1e9 / 122.88e6))
            rxb = dev.readSetting("RX_BANK_STATUS")
            egr = dev.readSetting("EGRESS_STATUS")
            print("  RX_BANK_STATUS: %s" % rxb)
            print("  EGRESS_STATUS:  %s" % egr)
            rec.append({"exp": "rx_probe", "activate_ret": act,
                        "reads": probe, "rx_bank": rxb, "egress": egr})
            dev.deactivateStream(rxs)
        if args.skip_a2:
            raise SystemExit(rc)
        print("== A2: abort while running, re-arm WITHOUT gate_release ==")
        dev.writeSetting("TDD_CMD", "abort")
        s = stat(dev)
        print("  after abort-on-running: gates_held=%s state=%s"
              % (s.get("gates_held"), s.get("state")))
        ok_a2, info_a2 = try_arm(dev, "A2-no-release")
        rec.append({"exp": "A2", "post_abort_stat": s, "armed": ok_a2,
                    "info": info_a2})
        if ok_a2:
            dev.writeSetting("TDD_CMD", "abort")

        print("== A3: full ladder, then re-arm (control) ==")
        full_ladder(dev)
        dev.writeSetting("TDD_SCHED", SOUNDER_SCHED)
        dev.writeSetting("TDD_REPLAY_STROBE", SOUNDER_STROBE_ON)
        ok_a3, info_a3 = try_arm(dev, "A3-ladder")
        rec.append({"exp": "A3", "armed": ok_a3, "info": info_a3})

    finally:
        print("== teardown: full ladder + strobe off + close streams ==")
        try:
            full_ladder(dev)
            dev.writeSetting("TDD_REPLAY_STROBE", "ch1:off")
        except Exception as e:  # noqa: BLE001
            print("  teardown step threw: %s" % e)
            rc = 3
        for h, name in ((rxs, "rx"), (txs, "tx")):
            if h is not None:
                try:
                    dev.closeStream(h)
                except Exception as e:  # noqa: BLE001
                    print("  closeStream(%s) threw: %s" % (name, e))
                    rc = rc or 3
        s = stat(dev)
        print("  final: state=%s gates_held=%s" % (s.get("state"),
                                                   s.get("gates_held")))
        if s.get("state") != "idle" or s.get("gates_held") != "0":
            rc = rc or 5
        rec.append({"exp": "teardown", "stat": s})
        with open(args.out, "w") as f:
            for r in rec:
                f.write(json.dumps(r, sort_keys=True) + "\n")
        print("wrote %d records to %s (rc=%d)" % (len(rec), args.out, rc))
    return rc


if __name__ == "__main__":
    sys.exit(main())
