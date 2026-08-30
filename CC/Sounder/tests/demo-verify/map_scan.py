#!/usr/bin/env python3
"""Channel-map scan: which BS TX channel reaches which UE RX channel?

For each BS TX chan t in {0,1}: load the real beacon RAM on t, arm the
slot-granular ring with the strobe on t. For each UE RX chan r in {0..3}:
read a few windows (local_port = 10001+r) and report window rms and the
find_beacon detector's best ratio in both conjugation senses. The cabled
pair shows a large rms and a huge ratio; leakage shows a weak ratio.
"""
import argparse
import sys
import time

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_TX

RATE = 122.88e6
SLOT = 4096
SPF = 30


def find_beacon_ratio(raw, gold):
    L = len(gold)
    n = 1 << int(np.ceil(np.log2(len(raw) + L)))
    gc = np.fft.ifft(np.fft.fft(raw, n) * np.conj(np.fft.fft(gold, n)))
    gc = gc[:len(raw) - L + 1]
    ac = np.zeros(len(gc), dtype=np.complex128)
    ac[L:] = gc[L:] * np.conj(gc[:-L])
    peak = np.abs(ac) ** 2
    ca = np.abs(gc) ** 2
    csum = np.concatenate(([0.0], np.cumsum(ca)))
    idx = np.arange(len(gc))
    thresh = csum[idx] - csum[np.maximum(0, idx - L)]
    return float((peak / (thresh + 1e-30)).max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bs-ip", default="168.6.244.21")
    ap.add_argument("--ue-ip", default="168.6.244.22")
    ap.add_argument("--core", default="beacon_core.bin")
    ap.add_argument("--gold", default="gold.bin")
    args = ap.parse_args()

    core = np.fromfile(args.core, dtype=np.int16)
    cc = core[0::2].astype(np.float64) - 1j * core[1::2]
    pk = np.abs(cc).max()
    ram = np.zeros(2 * 4096, dtype=np.int16)
    ram[0:2 * len(cc):2] = np.round(cc.real / pk * 0.6 * 32767).astype(np.int16)
    ram[1:2 * len(cc):2] = np.round(cc.imag / pk * 0.6 * 32767).astype(np.int16)
    gold = np.fromfile(args.gold, dtype=np.complex64).astype(np.complex128)

    bs = SoapySDR.Device(dict(driver="houdinisdr",
                              remote="tcp://%s:55132" % args.bs_ip,
                              timeout="1000000"))
    ue = SoapySDR.Device(dict(driver="houdinisdr",
                              remote="tcp://%s:55132" % args.ue_ip,
                              timeout="1000000"))
    buf = np.zeros(2 * 16384, dtype=np.int16)

    def ladder(d):
        d.writeSetting("TDD_CMD", "abort")
        d.writeRegister("RFCORE", 0x24, 1)
        d.writeRegister("RFCORE", 0x24, 0)
        d.writeSetting("TDD_CMD", "gate_release")

    try:
        for t in (0, 1):
            ladder(bs)
            for c in (0, 1):
                try:
                    bs.writeSetting("TDD_REPLAY_STROBE", "ch%d:off" % c)
                except Exception:  # noqa: BLE001
                    pass
            bs.setSampleRate(SOAPY_SDR_TX, t, RATE)
            bs.setFrequency(SOAPY_SDR_TX, t, 500e6)
            txs = bs.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [t],
                                 dict(tx_mode="replay"))
            r = bs.writeStream(txs, [ram], 4096)
            bs.writeSetting("TDD_SCHED", "6" + "2" * (SPF - 1))
            bs.writeSetting("TDD_REPLAY_STROBE",
                            "ch%d:len=1856,loops=1,offs=384" % t)
            bs.writeSetting("TDD_ARM",
                            "symbol_ticks=%d,symbols_per_frame=%d,"
                            "margin=36864000" % (SLOT, SPF))
            rb = dict(kv.split("=", 1) for kv in
                      bs.readSetting("TDD_ARM").split() if "=" in kv)

            def bank(ch):
                txt = bs.readSetting("TX_BANK_STATUS")
                for part in txt.split(";"):
                    if part.startswith("ch%d:" % ch):
                        return dict(kv.split("=", 1)
                                    for kv in part.split(":", 1)[1].split(","))
                return {}
            b0 = bank(t)
            time.sleep(1.0)
            b1 = bank(t)
            print("BS TX ch%d: ram=%d armed=%s played+%d late+%d smiss=%s"
                  % (t, r.ret, rb.get("accepted"),
                     int(b1["played"]) - int(b0["played"]),
                     int(b1["late"]) - int(b0["late"]), b1["smiss"]))

            for rx in range(4):
                try:
                    ue.setSampleRate(SOAPY_SDR_RX, rx, RATE)
                    ue.setFrequency(SOAPY_SDR_RX, rx, 500e6)
                    rxs = ue.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [rx],
                                         dict(local_port=str(10001 + rx),
                                              rx_gap_break="1"))
                    ue.activateStream(rxs)
                    best_r = 0.0
                    best_c = 0.0
                    rms = 0.0
                    amax = 0
                    got = 0
                    for _ in range(8):
                        sr = ue.readStream(rxs, [buf], 12288,
                                           timeoutUs=1000000)
                        if sr.ret <= 0:
                            continue
                        got += 1
                        w = buf[:2 * sr.ret]
                        cx = ((w[0::2].astype(np.float64) +
                               1j * w[1::2]) / 32767.0).astype(np.complex128)
                        rms = max(rms, float(np.sqrt(np.mean(np.abs(cx)**2))))
                        amax = max(amax, int(np.abs(w).max()))
                        best_r = max(best_r, find_beacon_ratio(cx, gold))
                        best_c = max(best_c,
                                     find_beacon_ratio(cx, np.conj(gold)))
                    ue.deactivateStream(rxs)
                    ue.closeStream(rxs)
                    print("  TX%d -> RX%d: windows=%d rms=%.1f absmax=%d "
                          "ratio gold=%.3g conj=%.3g"
                          % (t, rx, got, rms, amax, best_r, best_c))
                except Exception as e:  # noqa: BLE001
                    print("  TX%d -> RX%d: ERROR %s" % (t, rx, e))
            try:
                bs.closeStream(txs)
            except Exception:  # noqa: BLE001
                pass
    finally:
        try:
            ladder(bs)
            for c in (0, 1):
                try:
                    bs.writeSetting("TDD_REPLAY_STROBE", "ch%d:off" % c)
                except Exception:  # noqa: BLE001
                    pass
        except Exception as e:  # noqa: BLE001
            print("teardown: %s" % e)
    return 0


if __name__ == "__main__":
    sys.exit(main())
