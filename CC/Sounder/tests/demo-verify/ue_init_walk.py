#!/usr/bin/env python3
"""Replay the sounder's UE (client) init call sequence one call at a time and
dump the observable device state between calls.

Phase 5 instrument of DEMO_VERIFICATION.md. Makes EXACTLY the calls
Radio.cc makes when ClientRadioSet constructs the client radio, in the same
order with the same arguments, and nothing else:

    0 make        SoapySDR.Device(args as ClientRadioSet::init builds them)
    1 rate_rx     setSampleRate(RX, 1, 122.88e6)
    2 rate_tx     setSampleRate(TX, 1, 122.88e6)   <- the SH-335 pattern:
                  a TX rate write immediately before a tx_mode=stream setup
    3 freq_rx     setFrequency(RX, 1, 500e6)
    4 freq_tx     setFrequency(TX, 1, 500e6)
    5 stream_aux  setupStream(TX, [0], tx_mode=replay, mts) -- MTS membership
                  for DAC tile 0 (never activated), Radio.cc AP-23 order
    6 stream_tx   setupStream(TX, [1], tx_mode=stream, tdd=1, mts)
    7 stream_rx   setupStream(RX, [1], local_port=10002, rx_gap_break=1, mts)

Between calls it snapshots every readable setting plus the Soapy getters, so
each call's effect (or lack of one) is a recorded observation. The snapshot
BEFORE step 2 answers the SH-335 exposure question directly: if the TX rate
already reads 122.88e6 on a booted board, our write is a no-op and the
documented trigger pattern is not exercised.

Usage (on the rig host, houdini plugin path set):
    python3 ue_init_walk.py --ip <ue-ip> [--until N] [--out state.jsonl]

Exit hygiene: closes any streams it opened, never activates or arms anything,
and reports TDD_STAT at exit.
"""
import argparse
import json
import sys
import time

import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_TX

from bs_init_walk import READ_KEYS, snap, diff  # same registry + machinery

CH = 1  # ue_channel "B" -> physical channel 1
RATE = 122.88e6
NCO = 500e6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--port", default="55132")
    ap.add_argument("--until", type=int, default=7)
    ap.add_argument("--out", default="ue_init_walk.jsonl")
    args = ap.parse_args()

    make_args = dict(
        driver="houdinisdr",
        remote="tcp://%s:%s" % (args.ip, args.port),
        timeout="1000000",
    )
    make_args["remote:driver"] = "houdinisdr-device"
    make_args["remote:type"] = "houdinisdr"

    records = []
    rc = 0
    print("[0] make %s" % make_args)
    dev = SoapySDR.Device(make_args)
    aux = txs = rxs = None
    try:
        hi = dict(dev.getHardwareInfo())
        records.append({"_label": "hardware_info", "info": hi})
        print("    hardware_info: %s" % json.dumps(hi, sort_keys=True))
        cur = snap(dev, "after_make")
        records.append(cur)
        print("    baseline snapshot taken; boot TX rate ch%d = %s" %
              (CH, cur.get("tx_rate_ch%d" % CH)))

        def stream_aux():
            nonlocal aux
            aux = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [0],
                                  dict(tx_mode="replay", mts="true"))

        def stream_tx():
            nonlocal txs
            txs = dev.setupStream(
                SOAPY_SDR_TX, SOAPY_SDR_CS16, [CH],
                dict(tx_mode="stream", tdd="1", mts="true"))

        def stream_rx():
            nonlocal rxs
            rxs = dev.setupStream(
                SOAPY_SDR_RX, SOAPY_SDR_CS16, [CH],
                dict(local_port="10002", rx_gap_break="1", mts="true"))

        steps = [
            ("rate_rx", lambda: dev.setSampleRate(SOAPY_SDR_RX, CH, RATE)),
            ("rate_tx", lambda: dev.setSampleRate(SOAPY_SDR_TX, CH, RATE)),
            ("freq_rx", lambda: dev.setFrequency(SOAPY_SDR_RX, CH, NCO)),
            ("freq_tx", lambda: dev.setFrequency(SOAPY_SDR_TX, CH, NCO)),
            ("stream_aux_mts_ch0", stream_aux),
            ("stream_tx", stream_tx),
            ("stream_rx", stream_rx),
        ]
        for n, (label, fn) in enumerate(steps[: args.until], start=1):
            print("[%d] %s" % (n, label))
            t0 = time.time()
            try:
                fn()
                err = None
            except Exception as e:  # noqa: BLE001 - the throw IS the datum
                err = str(e)
                print("    THREW: %s" % err)
            nxt = snap(dev, "after_" + label)
            nxt["_call_s"] = round(time.time() - t0, 3)
            if err is not None:
                nxt["_threw"] = err
            d = diff(cur, nxt)
            records.append(nxt)
            print("    changed: %s" % (json.dumps(d, sort_keys=True) or "{}"))
            cur = nxt
    finally:
        for s, name in ((rxs, "rx"), (txs, "tx"), (aux, "aux")):
            if s is not None:
                try:
                    dev.closeStream(s)
                    print("closed %s stream" % name)
                except Exception as e:  # noqa: BLE001
                    print("closeStream(%s) threw: %s" % (name, e))
                    rc = 1
        try:
            tdd = dev.readSetting("TDD_STAT")
            print("exit TDD_STAT: %s" % tdd)
            records.append({"_label": "exit", "TDD_STAT": tdd})
        except Exception as e:  # noqa: BLE001
            print("exit TDD_STAT threw: %s" % e)
            rc = 1
        dev = None
    with open(args.out, "w") as f:
        for r in records:
            f.write(json.dumps(r, sort_keys=True) + "\n")
    print("wrote %s (%d records), rc=%d" % (args.out, len(records), rc))
    return rc


if __name__ == "__main__":
    sys.exit(main())
