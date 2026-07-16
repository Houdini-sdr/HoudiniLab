#!/usr/bin/env python3
"""
beacon_tdd.py -- beacon via TX replay GATED BY THE TDD FRAMER.

Models SoapyHoudiniSDR's host/tests/hil/test_tdd.py::test_tdd_replay_strobe_beacon
but standalone (no pytest) and with (a) our dual-NCO cable-band placement and
(b) a beacon-shaped replay payload + matched-filter readout.

How a beacon rides a TDD frame on Houdini:
  * TDD_SCHED is one hex char per symbol; the lane bits are {bit2 replay_strobe,
    bit1 rx_gate, bit0 tx_gate}. So '6' = replay_strobe+rx_gate = a BEACON symbol
    (fire the pre-loaded TX-replay burst AND capture); '2' = RX-only (silence
    control); '0' = guard.
  * Load the beacon into the replay RAM (writeStream, 2048 samples, amp 0.25 --
    the FIFO-safe recipe the passing tests use), then TDD_REPLAY_STROBE
    "ch{tx}:len,offs" arms it. TDD_ARM starts the framer, which fires exactly ONE
    beacon burst per frame at window_open+offs and re-arms autonomously.
  * We host-pump the '6' beacon window and the '2' silence window and check the
    burst energy lands in the beacon window and NOT the silence one; for chirp/zc
    beacons we also DDC + matched-filter the burst for a timing peak.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 beacon_tdd.py                       # single-board .21 DAC_B->ADC_D, tone
    python3 beacon_tdd.py --beacon chirp
    python3 beacon_tdd.py --tx-ip 168.6.244.21 --rx-ip 168.6.244.22 --rx-ch 2
"""
import argparse
import os
import sys
import time

import numpy as np

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
                           iq_from_cs16, tx_iq_tone, tx_lfm_chirp)

# TDD framer grid constants (from test_tdd.py).
GRID_TICKS = 384
QUANTUM_NS = 3125
SYM = 61440                 # symbol_ticks: 0.5 ms = 160*GRID_TICKS
ARM_MARGIN = 36_864_000     # ~300 ms of ticks: armed + pump lead
REG_TX_CLEAR_ALL = 0x24
ZC_U = 25


# ---- TDD framer control (writeSetting/readSetting surface) -------------------
def _ns_of_tick(t):
    assert t % GRID_TICKS == 0, f"tick {t} off grid"
    return (t // GRID_TICKS) * QUANTUM_NS


def _hw_tick(sdr, tick_rate):
    return int(round(int(sdr.getHardwareTime()) * tick_rate / 1e9))


def _arm(sdr, **kw):
    sdr.writeSetting("TDD_ARM", ",".join(f"{k}={v}" for k, v in kw.items()))
    out = {}
    for tok in sdr.readSetting("TDD_ARM").split():
        k, v = tok.split("=", 1)
        out[k] = v if k == "state" else int(v)
    return out


def _cmd(sdr, c):
    sdr.writeSetting("TDD_CMD", c)


def _teardown(sdr):
    try:
        _cmd(sdr, "abort")
        sdr.writeRegister("RFCORE", REG_TX_CLEAR_ALL, 1)
        sdr.writeRegister("RFCORE", REG_TX_CLEAR_ALL, 0)
        _cmd(sdr, "gate_release")
    except Exception as e:  # noqa: BLE001
        print(f"  [tdd] teardown warn: {e}")


def _next_window_tick(sdr, tick_rate, epoch, frame, sym_off=0, lead_frames=2):
    now = _hw_tick(sdr, tick_rate)
    k = max((now - epoch) // frame + lead_frames, 1)
    return epoch + k * frame + sym_off * SYM


# ---- beacon payloads ---------------------------------------------------------
def _zc(u, n):
    k = np.arange(n)
    return np.exp(-1j * np.pi * u * k * (k + 1) / n)


def build_beacon(kind, dac_rate, n_load, center_hz, amp, bw, n_sc):
    """Return (interleaved_int16, label) for the replay RAM at dac_rate."""
    if kind == "tone":
        i16, act = tx_iq_tone(center_hz, dac_rate, n_load, amp_frac=amp)
        return i16, f"tone @ {act/1e6:.3f} MHz"
    if kind == "chirp":
        i16, act = tx_lfm_chirp(bw, dac_rate, n_load, amp_frac=amp,
                                center_hz=center_hz)
        return i16, f"LFM chirp {act/1e6:.1f} MHz BW @ {center_hz/1e6:.1f} MHz"
    if kind == "zc":
        scs = dac_rate / n_load
        c = int(round(center_hz / scs))
        X = np.zeros(n_load, dtype=complex)
        X[(np.arange(n_sc) - n_sc // 2 + c) % n_load] = _zc(ZC_U, n_sc)
        x = np.fft.ifft(X) * n_load
        x = x / np.max(np.abs(x)) * amp
        i16 = np.zeros(2 * n_load, dtype=np.int16)
        i16[0::2] = np.round(x.real * 32767).astype(np.int16)
        i16[1::2] = np.round(x.imag * 32767).astype(np.int16)
        return i16, f"ZC-{n_sc} @ {center_hz/1e6:.1f} MHz ({n_sc*scs/1e6:.2f} MHz)"
    raise ValueError(kind)


def sliding_rms(x, w):
    """Peak of a length-w moving RMS over |x|, and its start index."""
    p = np.abs(x) ** 2
    csum = np.concatenate(([0.0], np.cumsum(p)))
    win = (csum[w:] - csum[:-w]) / w
    k = int(np.argmax(win))
    return float(np.sqrt(win[k])), k


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tx-ip", default="168.6.244.21")
    ap.add_argument("--rx-ip", default="168.6.244.21", help="=tx-ip -> loopback")
    ap.add_argument("--tx-ch", type=int, default=1, help="DAC_B = TX ch1")
    ap.add_argument("--rx-ch", type=int, default=3, help="ADC_D = RX ch3")
    ap.add_argument("--dac-nco", type=float, default=820.0)
    ap.add_argument("--adc-nco", type=float, default=388.8)
    ap.add_argument("--beacon", choices=["tone", "chirp", "zc"], default="tone")
    ap.add_argument("--center-mhz", type=float, default=20.0, help="beacon baseband centre")
    ap.add_argument("--bw-mhz", type=float, default=30.0, help="chirp bandwidth")
    ap.add_argument("--n-sc", type=int, default=4, help="zc subcarriers")
    ap.add_argument("--n-load", type=int, default=2048, help="replay RAM samples")
    ap.add_argument("--amp", type=float, default=0.25)
    ap.add_argument("--spf", type=int, default=100, help="symbols per frame")
    ap.add_argument("--beacon-sym", type=int, default=0)
    ap.add_argument("--silence-sym", type=int, default=50)
    a = ap.parse_args()

    loopback = a.rx_ip == a.tx_ip
    print(f"TX {a.tx_ip} ch{a.tx_ch} -> RX {a.rx_ip} ch{a.rx_ch}  "
          f"{'(single-board loopback)' if loopback else '(cross-board)'}  "
          f"beacon={a.beacon}")
    tx_ctx = hs.open_device(node=a.tx_ip, ch=a.tx_ch, verbose=False)
    rx_ctx = tx_ctx if loopback else hs.open_device(node=a.rx_ip, ch=a.rx_ch,
                                                    verbose=False)
    sdr = tx_ctx["sdr"]
    native, dtype = rx_ctx["native_fmt"], rx_ctx["dtype"]
    bps = rx_ctx["bytes_per_samp"]
    tick_rate = float(dict(sdr.getHardwareInfo()).get("tick_rate_hz", 122.88e6))

    _teardown(sdr)                                     # start clean
    ladder = list(sdr.listSampleRates(SOAPY_SDR_TX, a.tx_ch))
    if ladder:
        sdr.setSampleRate(SOAPY_SDR_TX, a.tx_ch, max(ladder))
    dac_rate = float(sdr.getSampleRate(SOAPY_SDR_TX, a.tx_ch))
    rx_rate = float(sdr.getSampleRate(SOAPY_SDR_RX, a.rx_ch))
    spt = rx_rate / tick_rate
    n_load = a.n_load
    offs = GRID_TICKS
    offs_rx = int(round(offs * spt))
    burst_rx = int(round(n_load * rx_rate / dac_rate))
    print(f"  dac_rate {dac_rate/1e6:.2f}  rx_rate {rx_rate/1e6:.2f}  "
          f"burst {burst_rx} samp @ offs {offs_rx}; NCO DAC {a.dac_nco} / "
          f"ADC {a.adc_nco} -> RF {a.dac_nco+a.center_mhz:.1f} MHz")

    tx = sdr.setupStream(SOAPY_SDR_TX, native, [a.tx_ch], {"tx_mode": "replay"})
    rx = sdr.setupStream(SOAPY_SDR_RX, native, [a.rx_ch], rx_stream_args(a.rx_ch))
    res = {}
    try:
        sdr.setFrequency(SOAPY_SDR_TX, a.tx_ch, a.dac_nco * 1e6)
        sdr.setFrequency(SOAPY_SDR_RX, a.rx_ch, a.adc_nco * 1e6)
        i16, label = build_beacon(a.beacon, dac_rate, n_load, a.center_mhz * 1e6,
                                  a.amp, a.bw_mhz * 1e6, a.n_sc)
        print(f"  replay payload: {label}, {n_load} samp, amp {a.amp}")
        cs16 = np.ascontiguousarray(i16, dtype=np.int16).view(np.int32)
        sdr.writeStream(tx, [cs16], n_load, 0, 0)      # a-temporal RAM load

        pattern = ["0"] * a.spf
        pattern[a.beacon_sym] = "6"                    # replay_strobe + rx_gate
        pattern[a.silence_sym] = "2"                   # rx_gate only (silence)
        sdr.writeSetting("TDD_SCHED", "".join(pattern))
        sdr.writeSetting("TDD_REPLAY_STROBE",
                         f"ch{a.tx_ch}:len={n_load // 2},offs={offs}")

        r = _arm(sdr, symbol_ticks=SYM, symbols_per_frame=a.spf, margin=ARM_MARGIN)
        assert r.get("accepted") == 1, f"TDD arm rejected: {r}"
        epoch, frame = r["epoch"], a.spf * SYM
        per_packet = rx_framing(sdr, verbose=False)["frame_words"] * (8 // bps)
        cap = (int(SYM * spt) // 2 // per_packet) * per_packet

        def pump(sym_off):
            ot = _next_window_tick(sdr, tick_rate, epoch, frame, sym_off)
            rc = sdr.activateStream(rx, SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST,
                                    _ns_of_tick(ot), cap)
            assert rc == 0, f"window arm rejected rc={rc}"
            out = run_burst(sdr, rx, per_packet, cap, dtype, buf_samples=256 * 1024)
            sdr.deactivateStream(rx)
            return iq_from_cs16(out["samples"]).astype(np.complex128)

        res["beacon"] = pump(a.beacon_sym)
        res["silence"] = pump(a.silence_sym)
        for key in ("TDD_STAT", "TX_BANK_STATUS"):
            try:
                print(f"  {key}: {sdr.readSetting(key)}")
            except Exception:  # noqa: BLE001
                pass
    finally:
        for s in (rx, tx):
            try:
                sdr.deactivateStream(s)
            except Exception:  # noqa: BLE001
                pass
            try:
                sdr.closeStream(s)
            except Exception:  # noqa: BLE001
                pass
        _teardown(sdr)

    # --- analysis: the burst is narrowband (invisible in broadband RMS), so DDC to
    #     the beacon band first -> the burst shows as a power spike in the beacon
    #     window at ~offs_rx and NOT in the silence window ---
    f0 = a.center_mhz * 1e6          # dual-NCO: received tone ~ +centre MHz
    bw = a.bw_mhz * 1e6 if a.beacon == "chirp" else 2.0e6

    def band_burst(iq):
        n = np.arange(len(iq))
        d = iq * np.exp(-2j * np.pi * f0 * n / rx_rate)
        k = max(1, int(round(rx_rate / bw)))         # boxcar LP ~ beacon bandwidth
        if k > 1:
            d = np.convolve(d, np.ones(k) / k, mode="same")
        return sliding_rms(d, max(16, burst_rx))

    b_rms, b_at = band_burst(res["beacon"])
    s_rms, _ = band_burst(res["silence"])
    floor = float(np.sqrt(np.mean(np.abs(res["beacon"]
                          * np.exp(-2j*np.pi*f0*np.arange(len(res["beacon"]))/rx_rate)
                          ) ** 2) + 1e-30))
    print(f"\nbeacon window (DDC to {f0/1e6:.1f} MHz): burst RMS {b_rms:6.2f} at samp "
          f"{b_at} (expected ~{offs_rx}); band floor {floor:.2f}")
    print(f"silence window: burst RMS {s_rms:6.2f}")
    burst_db = 20 * np.log10(b_rms / max(s_rms, 1e-6))
    print(f"beacon/silence = {burst_db:.1f} dB")

    fired = burst_db >= 8.0 and abs(b_at - offs_rx) <= 2 * burst_rx
    print("\nRESULT:",
          f"BEACON FIRED in the TDD '6' slot @ samp {b_at} ({burst_db:.1f} dB over "
          f"the silence window) -- replay strobe + TDD framer OK"
          if fired else
          f"no clean beacon burst in the '6' slot (beacon/silence {burst_db:.1f} dB, "
          f"at {b_at} vs expected {offs_rx})")
    return 0 if fired else 1


if __name__ == "__main__":
    sys.exit(main())
