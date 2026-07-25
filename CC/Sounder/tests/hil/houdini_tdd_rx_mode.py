#!/usr/bin/env python3
"""
houdini_tdd_rx_mode.py -- does the BS RX MODE disrupt the beacon strobe?

.21 runs a TDD framer: beacon replay strobe (loops=forever) @ symbol 0 + rx_gate @
symbol 1. While the beacon fires, .21 also runs its RX in one of three modes (a
background thread), and we measure the beacon on .22 (single-window matched filter,
the client's detection mode):

  none        no .21 RX at all (baseline; = beacon_tdd_xboard's ~33 dB)
  continuous  activate .21 RX ONCE, read the gated stream in a loop (the correct
              TDD model -- what houdiniTddRx SHOULD do)
  pump        per-window activate(HAS_TIME)->read->deactivate on .21 (what
              houdiniTddRx does NOW; TDD_TEST_MATRIX D7a flags this as disrupting TX)

If continuous ~ none >> pump, the per-window pump is the beacon-killer and the fix is
to activate .21 RX once + read the gated stream.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 houdini_tdd_rx_mode.py --rx-mode none
    python3 houdini_tdd_rx_mode.py --rx-mode continuous
    python3 houdini_tdd_rx_mode.py --rx-mode pump
"""
import argparse
import os
import sys
import threading

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
_EX = os.environ.get(
    "HOUDINI_EXAMPLES",
    os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))
if _EX not in sys.path:
    sys.path.insert(0, _EX)

import SoapySDR  # noqa: E402
from SoapySDR import (SOAPY_SDR_RX, SOAPY_SDR_TX, SOAPY_SDR_HAS_TIME,  # noqa: E402
                      SOAPY_SDR_END_BURST)
import houdini_setup as hs  # noqa: E402
from houdini_setup import (run_burst, rx_stream_args, rx_framing,  # noqa: E402
                           tx_lfm_chirp)
from beacon_tdd import (build_beacon, _arm, _teardown, _ns_of_tick,  # noqa: E402
                        _hw_tick, _next_window_tick, GRID_TICKS, SYM, ARM_MARGIN)


def matched_filter(x, h):
    n = 1 << int(np.ceil(np.log2(len(x) + len(h))))
    C = np.fft.ifft(np.fft.fft(x, n) * np.conj(np.fft.fft(h, n)))
    return np.abs(C[:len(x) - len(h) + 1])


def to_complex(lanes):
    try:
        return np.asarray(hs.iq_from_lanes(lanes, "interleaved"), dtype=np.complex128)
    except Exception:  # noqa: BLE001
        L = lanes.astype(np.float64)
        q = L[:, 1] if lanes.shape[1] < 3 else L[:, 2]
        return L[:, 0] + 1j * q


def bs_rx_worker(bsd, ch, native, dtype, mode, tick_rate, epoch, frame, pilot_sym,
                 stop):
    """Run .21 RX in `mode` until stop is set."""
    if mode == "none":
        return
    rx = bsd.setupStream(SOAPY_SDR_RX, native, [ch], rx_stream_args(ch))
    per_packet = rx_framing(bsd, verbose=False)["frame_words"] * (8 // (4 if dtype == np.int16 else 2))
    win = 4096
    try:
        if mode == "continuous":
            bsd.activateStream(rx)               # activate ONCE
            buf = np.zeros(2 * 65536, dtype=np.int16)
            while not stop.is_set():
                bsd.readStream(rx, [buf], 65536, timeoutUs=100000)  # drain the gated stream
        elif mode == "pump":
            spt = 1.0
            while not stop.is_set():             # per-window activate/read/deactivate
                ot = _next_window_tick(bsd, tick_rate, epoch, frame, pilot_sym)
                if bsd.activateStream(rx, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                                      _ns_of_tick(ot), win) != 0:
                    bsd.deactivateStream(rx)
                    continue
                run_burst(bsd, rx, per_packet, win, dtype, buf_samples=256 * 1024)
                bsd.deactivateStream(rx)
    finally:
        try:
            bsd.deactivateStream(rx)
        except Exception:  # noqa: BLE001
            pass
        bsd.closeStream(rx)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bs-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.22")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--nco", type=float, default=500.0)
    ap.add_argument("--center-mhz", type=float, default=20.0)
    ap.add_argument("--bw-mhz", type=float, default=30.0)
    ap.add_argument("--n-load", type=int, default=2048)
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--spf", type=int, default=2, help="1 ms frame like the sounder")
    ap.add_argument("--pilot-sym", type=int, default=1)
    ap.add_argument("--secs", type=float, default=0.3)
    ap.add_argument("--cap-mb", type=float, default=40.0)
    ap.add_argument("--rx-mode", choices=["none", "continuous", "pump"], default="none")
    a = ap.parse_args()

    print(f"BS {a.bs_ip} [beacon strobe@0 loops=forever + rx_gate@{a.pilot_sym}], "
          f".21 RX mode = {a.rx_mode.upper()}  ->  detect beacon on {a.rx_ip}")
    bs = hs.open_device(node=a.bs_ip, ch=a.tx_ch, verbose=False)
    rx = hs.open_device(node=a.rx_ip, ch=a.rx_ch, verbose=False)
    bsd, rsd = bs["sdr"], rx["sdr"]
    native, dtype = bs["native_fmt"], bs["dtype"]
    rnative, rdtype = rx["native_fmt"], rx["dtype"]
    tick_rate = float(dict(bsd.getHardwareInfo()).get("tick_rate_hz", 122.88e6))

    _teardown(bsd)
    ladder = list(bsd.listSampleRates(SOAPY_SDR_TX, a.tx_ch))
    if ladder:
        bsd.setSampleRate(SOAPY_SDR_TX, a.tx_ch, max(ladder))
    dac_rate = float(bsd.getSampleRate(SOAPY_SDR_TX, a.tx_ch))
    bsd.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.nco * 1e6)
    bsd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, 122.88e6)
    bsd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
    rsd.setSampleRate(SOAPY_SDR_RX, a.rx_ch, 122.88e6)
    rsd.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.nco * 1e6)
    rx_rate = float(rsd.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
    frame = a.spf * SYM
    burst_rx = int(round(a.n_load * rx_rate / dac_rate))

    i16, _ = tx_lfm_chirp(a.bw_mhz * 1e6, dac_rate, a.n_load, amp_frac=a.amp,
                          center_hz=a.center_mhz * 1e6)
    mi16, _ = tx_lfm_chirp(a.bw_mhz * 1e6, rx_rate, burst_rx, amp_frac=1.0, center_hz=0.0)
    match = (mi16[0::2].astype(np.float64) + 1j * mi16[1::2]).astype(np.complex128)
    match /= np.sqrt(np.sum(np.abs(match) ** 2)) + 1e-30

    tx = bsd.setupStream(SOAPY_SDR_TX, native, [a.tx_ch], {"tx_mode": "replay"})
    stop = threading.Event()
    worker = None
    try:
        cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)
        bsd.writeStream(tx, [cs16], a.n_load, 0, 0)
        pat = ["0"] * a.spf
        pat[0] = "6"                                  # beacon strobe + rx
        pat[a.pilot_sym] = "2"                        # rx_gate
        bsd.writeSetting("TDD_SCHED", "".join(pat))
        bsd.writeSetting("TDD_REPLAY_STROBE",
                         f"ch{a.tx_ch}:len={a.n_load//2},loops=forever,offs={GRID_TICKS}")
        r = _arm(bsd, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        assert r.get("accepted") == 1, f"arm rejected: {r}"
        epoch = r["epoch"]

        # start .21 RX in the chosen mode
        worker = threading.Thread(target=bs_rx_worker, args=(
            bsd, a.rx_ch, native, dtype, a.rx_mode, tick_rate, epoch, frame,
            a.pilot_sym, stop), daemon=True)
        worker.start()

        # detect the beacon on .22 (single-window = the client's mode)
        buf, _s = hs.capture_rx(rsd, a.rx_ch, rnative, rdtype, duration_sec=a.secs,
                                capture_bytes=int(a.cap_mb * 1024 * 1024))
        iq = to_complex(hs.cs16_lanes(buf))
        n = np.arange(len(iq))
        ddc = iq * np.exp(+2j * np.pi * a.center_mhz * 1e6 * n / rx_rate)
        c0 = matched_filter(ddc, match); c1 = matched_filter(ddc, np.conj(match))
        corr = c0 if c0.max() >= c1.max() else c1
        single = 20 * np.log10(corr.max() / (np.median(corr) + 1e-30))
        try:
            bank = bsd.readSetting("TX_BANK_STATUS")
            acked = [c for c in bank.split(";") if c.startswith(f"ch{a.tx_ch}:")]
            ack_str = acked[0].split(":", 1)[1][:40] if acked else "?"
        except Exception:  # noqa: BLE001
            ack_str = "?"
    finally:
        stop.set()
        if worker is not None:
            worker.join(timeout=5)
        try:
            bsd.writeSetting("TDD_REPLAY_STROBE", f"ch{a.tx_ch}:off")
        except Exception:  # noqa: BLE001
            pass
        for s in (tx,):
            try:
                bsd.deactivateStream(s); bsd.closeStream(s)
            except Exception:  # noqa: BLE001
                pass
        _teardown(bsd)

    print(f"\nRESULT [rx-mode={a.rx_mode}]: beacon single-window peak = {single:.1f} dB "
          f"over median   (TX bank: {ack_str})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
