#!/usr/bin/env python3
"""Replay the sounder's BS init call sequence one call at a time and dump the
observable device state between calls.

This is the phase 2 instrument of DEMO_VERIFICATION.md. It makes EXACTLY the
calls Radio.cc:139-198 makes for the base station, in the same order with the
same arguments, and nothing else. Between calls it snapshots every readable
setting plus the Soapy getters, so each call's effect (or lack of one) is a
recorded observation rather than an assumption.

Usage (on the rig host, inside the houdini venv):
    python3 bs_init_walk.py --ip 168.6.244.21 [--until N] [--out state.jsonl]

Steps:
    0 make        SoapySDR.Device(args as BaseRadioSet::init builds them)
    1 rate_rx     setSampleRate(RX, 1, 122.88e6)
    2 rate_tx     setSampleRate(TX, 1, 122.88e6)
    3 freq_rx     setFrequency(RX, 1, 500e6)
    4 freq_tx     setFrequency(TX, 1, 500e6)
    5 stream_rx   setupStream(RX, CS16, [1], local_port=10002, rx_gap_break=1)
    6 stream_tx   setupStream(TX, CS16, [1], tx_mode=replay)

Exit hygiene: closes any streams it opened, never arms anything, and reports
TDD_STAT at exit. Exits nonzero if teardown left visibly dirty state.
"""
import argparse
import json
import sys
import time

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

# Readable keys from the device setting registry (SoapyHoudiniSDR
# device/SoapyHoudiniSDR_setting_registry.cpp, spot-checked 2026-08-30).
READ_KEYS = [
    "HOUDINI_PROTO_VERSION",
    "HOUDINI_TICK_RATE",
    "RX_FAB_CLK",
    "TX_FAB_CLK",
    "HOUDINI_MTU",
    "HOUDINI_RX_FRAME_WORDS",
    "HOUDINI_RX_PAYLOAD_BYTES",
    "HOUDINI_RX_FIFO_HWM",
    "HOUDINI_RX_TARGET_LATENCY",
    "HOUDINI_RFDC_INTR_STATUS",
    "TX_BANK_STATUS",
    "TX_BUF_WATERMARK",
    "TX_EMIT_LEAD",
    # TX_REPLAY_RANGE is write-side only: reading it warns "unknown key" and
    # returns "" (observed 2026-08-30) -- the registry's read row is absent.
    "TDD_STAT",
    "TDD_ARM",
    "RX_BANK_STATUS",
    "EGRESS_STATUS",
    "HOUDINI_FPGA_TX_PORT",
    "HOUDINI_FPGA_RX_PORT",
    "HOUDINI_FPGA_DATA_IFACE",
]

CH = 1  # bs_channel "B" -> physical channel 1 (utils.cc strToChannels)
RATE = 122.88e6
NCO = 500e6


def snap(dev, label, ch=CH):
    s = {"_label": label, "_t": round(time.time(), 3)}
    for k in READ_KEYS:
        try:
            s[k] = dev.readSetting(k)
        except Exception as e:  # noqa: BLE001 - record, don't die
            s[k] = "<threw: %s>" % e
    for d, name in ((SOAPY_SDR_RX, "rx"), (SOAPY_SDR_TX, "tx")):
        for getter, gname in (
            (dev.getSampleRate, "rate"),
            (dev.getFrequency, "freq"),
        ):
            key = "%s_%s_ch%d" % (name, gname, ch)
            try:
                s[key] = getter(d, ch)
            except Exception as e:  # noqa: BLE001
                s[key] = "<threw: %s>" % e
    try:
        s["hw_time_ns"] = dev.getHardwareTime("")
    except Exception as e:  # noqa: BLE001
        s["hw_time_ns"] = "<threw: %s>" % e
    return s


def diff(prev, cur):
    out = {}
    for k, v in cur.items():
        if k.startswith("_") or k in ("hw_time_ns",):
            continue
        if prev.get(k) != v:
            out[k] = {"was": prev.get(k), "now": v}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--port", default="55132")
    ap.add_argument("--until", type=int, default=6)
    ap.add_argument("--out", default="bs_init_walk.jsonl")
    args = ap.parse_args()

    make_args = dict(
        driver="houdinisdr",
        remote="tcp://%s:%s" % (args.ip, args.port),
        timeout=RPC_TIMEOUT_US,
    )
    make_args["remote:driver"] = "houdinisdr-device"
    make_args["remote:type"] = "houdinisdr"

    records = []
    rc = 0
    print("[0] make %s" % make_args)
    dev = SoapySDR.Device(make_args)
    try:
        hi = dict(dev.getHardwareInfo())
        records.append({"_label": "hardware_info", "info": hi})
        print("    hardware_info: %s" % json.dumps(hi, sort_keys=True))
        cur = snap(dev, "after_make")
        records.append(cur)
        print("    baseline snapshot taken")

        steps = [
            ("rate_rx", lambda: dev.setSampleRate(SOAPY_SDR_RX, CH, RATE)),
            ("rate_tx", lambda: dev.setSampleRate(SOAPY_SDR_TX, CH, RATE)),
            ("freq_rx", lambda: dev.setFrequency(SOAPY_SDR_RX, CH, NCO)),
            ("freq_tx", lambda: dev.setFrequency(SOAPY_SDR_TX, CH, NCO)),
        ]
        rxs = txs = None

        def stream_rx():
            nonlocal rxs
            rxs = dev.setupStream(
                SOAPY_SDR_RX, SOAPY_SDR_CS16, [CH],
                dict(local_port="10002", rx_gap_break="1"))

        def stream_tx():
            nonlocal txs
            txs = dev.setupStream(
                SOAPY_SDR_TX, SOAPY_SDR_CS16, [CH], dict(tx_mode="replay"))

        steps += [("stream_rx", stream_rx), ("stream_tx", stream_tx)]

        for n, (label, fn) in enumerate(steps, start=1):
            if n > args.until:
                break
            print("[%d] %s" % (n, label))
            t0 = time.time()
            try:
                fn()
                err = None
            except Exception as e:  # noqa: BLE001
                err = str(e)
                print("    THREW: %s" % err)
            nxt = snap(dev, "after_%s" % label)
            nxt["_call_error"] = err
            nxt["_call_ms"] = round((time.time() - t0) * 1e3, 1)
            d = diff(cur, nxt)
            print("    delta: %s" % (json.dumps(d, sort_keys=True) if d
                                     else "(no observable change)"))
            records.append(nxt)
            cur = nxt
            if err is not None:
                rc = 2
                break

        # exit hygiene: close what we opened, then confirm the framer state
        for h, name in ((rxs, "rx"), (txs, "tx")):
            if h is not None:
                try:
                    dev.closeStream(h)
                    print("[teardown] closed %s stream" % name)
                except Exception as e:  # noqa: BLE001
                    print("[teardown] closeStream(%s) THREW: %s" % (name, e))
                    rc = 3
        fin = snap(dev, "after_teardown")
        records.append(fin)
        print("[teardown] TDD_STAT: %s" % fin.get("TDD_STAT"))
    finally:
        # the python binding's __del__ closes the device; an explicit unmake
        # here makes __del__ raise "unknown device" at interpreter exit
        with open(args.out, "w") as f:
            for r in records:
                f.write(json.dumps(r, sort_keys=True) + "\n")
        print("wrote %d records to %s" % (len(records), args.out))
    return rc


if __name__ == "__main__":
    sys.exit(main())
