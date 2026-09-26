#!/usr/bin/env python3
"""Two-node beacon arrival measurement (DEMO_VERIFICATION.md section 4).

BS (.21) is armed exactly as the slot-granular sounder does, with the REAL
beacon waveform (conjugated 496-sample core at 0.6 FS, zeros to 4096). The UE
(.22) runs the acquisition model [user 2026-08-30]: hunt all time for the gold
correlation; on the first match, jump (by tick arithmetic) to the next frame's
predicted beacon and confirm; two matches = timing established; then track.

Measurements:
  M1  hunt-to-lock: windows consumed to first + confirming match;
  M2  tracking: arrival residual (measured - predicted) per frame over N
      matches -- spread = arrival jitter; slope = residual clock drift;
  M3  re-arm determinism: K BS re-arms; each predicts its arrival-phase shift
      from (epoch_new - epoch_old) mod 122880; report |measured - predicted|;
  M4  re-make variability: M full BS device close/reopen/re-arm cycles (tile
      bring-up in between); same predicted-vs-measured comparison. This is
      the no-MTS bring-up variability probe (ledger row 2.18).

The UE never transmits. Data content beyond the correlation is not examined.
"""
import argparse
import json
import sys
import time

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_TX

# SoapyRemote's `timeout` device arg is MICROSECONDS, and it bounds the
# make() RPC. Measured 2026-09-01 on this bench: a COLD make (the server
# holds no live device instance, so construction runs the full RFDC
# bring-up) takes 3.34 s; a WARM one 0.34 s. The long-standing 1000000
# (= 1 s) therefore sits INSIDE the normal spread, and make() failed with
# "SoapyRPCUnpacker::recv() TIMEOUT" three times in one session depending
# only on whether a previous run still held the instance. That is a slow
# call against a short deadline, NOT an unresponsive server -- do not
# read it as one.
RPC_TIMEOUT_US = "30000000"

RATE = 122.88e6
FRAME = 122880
SLOT = 4096
SPF = 30
GOLD_L = 128
CORE_OFF_2NDREP = 368  # detector index = start of 2nd gold rep = core + 368


def ns_to_ticks(t_ns):
    return int(round(t_ns * RATE / 1e9))


def find_beacon(raw, gold, corr_scale):
    """Replica of CommsLib::find_beacon_avx (tests/hil/find_beacon_py.py)."""
    L = len(gold)
    if len(raw) < 2 * L + 8:
        return -1, 0.0
    n = 1 << int(np.ceil(np.log2(len(raw) + L)))
    gc = np.fft.ifft(np.fft.fft(raw, n) * np.conj(np.fft.fft(gold, n)))
    gc = gc[:len(raw) - L + 1]
    ac = np.zeros(len(gc), dtype=np.complex128)
    ac[L:] = gc[L:] * np.conj(gc[:-L])
    peak = np.abs(ac) ** 2
    ca = np.abs(gc) ** 2
    csum = np.concatenate(([0.0], np.cumsum(ca)))
    idx = np.arange(len(gc))
    lo = np.maximum(0, idx - L)
    thresh = csum[idx] - csum[lo]
    ratio = peak / (thresh + 1e-30)
    valid = np.where(corr_scale * peak > thresh)[0]
    if len(valid) == 0:
        return -1, float(ratio.max())
    # Single-copy beacon: no copy ambiguity, so take the GLOBAL argmax of the
    # peak metric. The C++ earliest-crossing rule can anchor ~430 samples
    # early on a threshold crossing inside the 16-periodic STS region (lag-128
    # self-coherent), with tens of samples of scatter between contexts --
    # measured 2026-08-30, see the ledger. The gold 2-rep argmax is sharp and
    # context-independent.
    best = int(np.argmax(peak))
    if corr_scale * peak[best] <= thresh[best]:
        best = int(valid[np.argmax(ratio[valid])])
    return int(best), float(ratio[best])


def ident(dev, want_ip, role):
    """Print and verify which physical board this handle is."""
    hi = dict(dev.getHardwareInfo())
    label = hi.get("label", "?")
    ipaddr = hi.get("ip_address", "")
    ok = want_ip in ipaddr
    print("  %s = %s (%s)%s" % (role, label, ipaddr,
                                "" if ok else "  MISMATCH, wanted %s" % want_ip))
    if not ok:
        raise RuntimeError("%s handle is %s, wanted %s" % (role, ipaddr,
                                                           want_ip))
    return label


class Ue:
    def __init__(self, ip, ch=1, dev=None):
        if dev is not None:
            self.dev = dev   # shared single-board mode; rates preset and the
            self.rxs = None  # caller sets up + activates the RX stream
        else:
            self.dev = SoapySDR.Device(dict(driver="houdinisdr",
                                            remote="tcp://%s:55132" % ip,
                                            timeout=RPC_TIMEOUT_US))
            ident(self.dev, ip, "UE")
            self.dev.setSampleRate(SOAPY_SDR_RX, ch, RATE)
            tune(self.dev, SOAPY_SDR_RX, ch, 500e6)
            self.rxs = self.dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [ch],
                                            dict(local_port=str(10001 + ch),
                                                 rx_gap_break="1"))
            act = self.dev.activateStream(self.rxs)
            print("  UE activateStream ret=%d" % act)
        self.buf = np.zeros(2 * 16384, dtype=np.int16)
        self.reads = 0
        self.fails = 0

    def window(self, n=12288):
        # Drain-first, the recvHoudini pattern: python consumes far slower
        # than the stream produces, so without a drain every read returns
        # ever-older backlog and (12288 | 122880) phase-locks the window comb
        # off the beacon forever. The data plane is an in-process call, so
        # the drain is cheap.
        for _ in range(600):  # bounded drain: ~10 MB, keeps windows cheap
            dr = self.dev.readStream(self.rxs, [self.buf], 16384, timeoutUs=0)
            if dr.ret <= 0:
                break
        sr = self.dev.readStream(self.rxs, [self.buf], n, timeoutUs=1000000)
        self.reads += 1
        if sr.ret <= 0:
            self.fails += 1
            if self.fails <= 3 or self.fails % 100 == 0:
                print("  UE read #%d ret=%d flags=0x%x (fail %d)"
                      % (self.reads, sr.ret, sr.flags, self.fails))
            return None, None
        w = self.buf[:2 * sr.ret]
        c = ((w[0::2].astype(np.float64) + 1j * w[1::2]) / 32767.0).astype(np.complex128)
        return ns_to_ticks(sr.timeNs), c

    def close(self):
        try:
            self.dev.deactivateStream(self.rxs)
            self.dev.closeStream(self.rxs)
        except Exception:  # noqa: BLE001
            pass


# HOW THE RADIO IS TUNED. The plugin exposes ONE tunable element, "RF", whose
# range is +-983.04 MHz: it is the RFDC NCO, and there is no "BB" element
# (asked 2026-09-03: "unknown tunable element 'BB'"). The sounder's Houdini
# path (Radio.cc constructor) makes exactly these calls -- setSampleRate then a
# plain setFrequency(dir, ch, nco) -- so the probes and the sounder tune the
# radios identically. The Iris-era "RF"/"BB" split in Radio::dev_init is not
# reached on Houdini.
STREAM_MTS = [True]
MTS_POWER_CH0 = [False]


STREAM_MTS_RX = [False]


def rx_stream_args(ch):
    # The UE's RX stream joins the MTS group only when asked: the driver's
    # first-up rule needs a DAC0/TX stream on the SAME device set up with
    # mts=true first, which the sounder's UE has (its TX stream) and this
    # capture-only probe does not.
    args = dict(local_port=str(10001 + ch), rx_gap_break="1")
    if STREAM_MTS[0] and STREAM_MTS_RX[0]:
        args["mts"] = "true"
    return args


def tune(dev, direction, ch, freq_hz):
    dev.setFrequency(direction, ch, freq_hz)


class Bs:
    def __init__(self, ip, ram_iq, ch=1):
        self.ip = ip
        self.ch = ch
        self.ram_iq = ram_iq
        self.dev = None
        self.rxs = None
        self.txs = None
        self.epoch = None

    def liveness(self, settle=1.0):
        """Beacon heartbeat: acked/played must advance ~1000/s, late must not."""
        def bank():
            txt = self.dev.readSetting("TX_BANK_STATUS")
            for part in txt.split(";"):
                if part.startswith("ch%d:" % self.ch):
                    return dict(kv.split("=", 1)
                                for kv in part.split(":", 1)[1].split(","))
            return {}
        b0 = bank()
        time.sleep(settle)
        b1 = bank()
        da = int(b1["acked"]) - int(b0["acked"])
        dp = int(b1["played"]) - int(b0["played"])
        dl = int(b1["late"]) - int(b0["late"])
        print("  BS liveness: acked+%d played+%d late+%d over %.1fs"
              % (da, dp, dl, settle))
        return dp > 0 and dl == 0

    def open_and_arm(self, dev=None, tx_only=True):
        if dev is not None:
            self.dev = dev          # shared single-board mode; the UE's live
            self.shared = True      # RX stream provides the preamble element
        else:
            self.shared = False
            self.dev = SoapySDR.Device(dict(driver="houdinisdr",
                                            remote="tcp://%s:55132" % self.ip,
                                            timeout=RPC_TIMEOUT_US))
            ident(self.dev, self.ip, "BS")
            # EXACTLY the sounder's order (Radio.cc constructor): rate and
            # NCO on the data channel, then the streams -- with MTS, a
            # never-activated ch0 replay stream FIRST for tile-0 membership
            # (the plugin's first-up rule), then the data TX stream, then RX.
            # The TDD ladder and the RAM load come after, as in BaseRadioSet.
            self.dev.setSampleRate(SOAPY_SDR_TX, self.ch, RATE)
            tune(self.dev, SOAPY_SDR_TX, self.ch, 500e6)
            if not tx_only:
                self.dev.setSampleRate(SOAPY_SDR_RX, self.ch, RATE)
                tune(self.dev, SOAPY_SDR_RX, self.ch, 500e6)
        txargs = dict(tx_mode="replay")
        self.aux_txs = None
        if STREAM_MTS[0]:
            txargs["mts"] = "true"
            if self.ch != 0 and not self.shared:
                if MTS_POWER_CH0[0]:
                    # Variant under test: give tile 0 a rate and NCO before it
                    # is asked to join the MTS group.
                    self.dev.setSampleRate(SOAPY_SDR_TX, 0, RATE)
                    tune(self.dev, SOAPY_SDR_TX, 0, 500e6)
                self.aux_txs = self.dev.setupStream(
                    SOAPY_SDR_TX, SOAPY_SDR_CS16, [0], dict(tx_mode="replay", mts="true"))
        self.txs = self.dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [self.ch], txargs)
        if not self.shared and not tx_only:
            self.rxs = self.dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [self.ch],
                                            rx_stream_args(self.ch))
        # map_scan's exact pre-arm cleanup: FULL ladder + both strobes off
        # (the arrival flow's abort-only differed; bisecting the weak-RF gap)
        self.ladder()
        for c in (0, 1):
            try:
                self.dev.writeSetting("TDD_REPLAY_STROBE", "ch%d:off" % c)
            except Exception:  # noqa: BLE001
                pass
        r = self.dev.writeStream(self.txs, [self.ram_iq], 4096)
        if r.ret != 4096:
            raise RuntimeError("RAM load ret=%d" % r.ret)
        self.arm()

    def arm(self):
        self.dev.writeSetting("TDD_SCHED", "6" + "2" * (SPF - 1))
        self.dev.writeSetting("TDD_REPLAY_STROBE",
                              "ch%d:len=1856,loops=1,offs=384" % self.ch)
        self.dev.writeSetting(
            "TDD_ARM", "symbol_ticks=%d,symbols_per_frame=%d,margin=36864000"
            % (SLOT, SPF))
        rb = dict(kv.split("=", 1) for kv in
                  self.dev.readSetting("TDD_ARM").split() if "=" in kv)
        if rb.get("accepted") != "1":
            raise RuntimeError("arm not accepted: %s" % rb)
        self.epoch = int(rb["epoch"])

    def ladder(self):
        self.dev.writeSetting("TDD_CMD", "abort")
        self.dev.writeRegister("RFCORE", 0x24, 1)
        self.dev.writeRegister("RFCORE", 0x24, 0)
        self.dev.writeSetting("TDD_CMD", "gate_release")

    def rearm(self):
        self.ladder()
        try:
            self.dev.writeSetting("TDD_REPLAY_STROBE", "ch%d:off" % self.ch)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.dev.deactivateStream(self.txs)  # rewinds the host fill cursor
        except Exception:  # noqa: BLE001
            pass
        r = self.dev.writeStream(self.txs, [self.ram_iq], 4096)
        if r.ret != 4096:
            raise RuntimeError("re-load ret=%d" % r.ret)
        self.arm()

    def close(self):
        if self.dev is None:
            return
        try:
            self.ladder()
            self.dev.writeSetting("TDD_REPLAY_STROBE", "ch%d:off" % self.ch)
        except Exception:  # noqa: BLE001
            pass
        if getattr(self, "shared", False):
            # shared device: only release the TX stream; the caller owns dev
            if self.txs is not None:
                try:
                    self.dev.closeStream(self.txs)
                except Exception:  # noqa: BLE001
                    pass
            if getattr(self, "aux_txs", None) is not None:
                try:
                    self.dev.closeStream(self.aux_txs)
                except Exception:  # noqa: BLE001
                    pass
                self.aux_txs = None
            self.dev = None
            self.txs = None
            return
        for h in (self.txs, self.rxs):
            if h is not None:
                try:
                    self.dev.closeStream(h)
                except Exception:  # noqa: BLE001
                    pass
        self.dev = None
        self.rxs = None
        self.txs = None


DEADLINE = [None]
MIN_RATIO = [1e4]


def over_budget():
    return DEADLINE[0] is not None and time.time() > DEADLINE[0]


def acquire(ue, gold, corr_scale, max_windows=400):
    """Hunt -> first match -> predict next frame -> confirm. Returns
    (anchor_tick, sense_name, windows_used) or (None, None, n)."""
    senses = [("gold", gold), ("conj", np.conj(gold))]
    first = None
    sense = None
    best = {"gold": 0.0, "conj": 0.0}
    confirms = 0
    for w in range(max_windows):
        if over_budget():
            print("  (budget) acquire stops at w=%d" % w)
            break
        tk, c = ue.window()
        if tk is None:
            continue
        if first is None:
            for name, g in senses:
                idx, ratio = find_beacon(c, g, corr_scale)
                best[name] = max(best[name], ratio)
                if idx >= 0 and ratio < MIN_RATIO[0]:
                    idx = -1  # noise-window artifact, not the beacon
                if idx >= 0:
                    first = tk + idx - CORE_OFF_2NDREP
                    sense = (name, g)
                    rms = float(np.sqrt(np.mean(np.abs(c) ** 2)))
                    print("  first-match w=%d sense=%s idx=%d ratio=%.2f "
                          "win_rms=%.1f" % (w, name, idx, ratio, rms))
                    break
            if first is None and (w + 1) % 50 == 0:
                rms = float(np.sqrt(np.mean(np.abs(c) ** 2)))
                print("  hunt w=%d rms=%.1f best_ratio gold=%.3g conj=%.3g"
                      % (w + 1, rms, best["gold"], best["conj"]))
            continue
        # confirm: does THIS window contain the predicted arrival?
        name, g = sense
        k = int(round((tk + 2048 - first) / FRAME))
        pred = first + k * FRAME
        off = pred - tk
        if confirms == 0 and w < 12:
            print("  confirm-scan w=%d ret=%d off=%d" % (w, len(c), off))
        if not (0 <= off < len(c) - 700):
            continue
        sl = c[max(0, off - 256):off + 496 + 384]
        idx, ratio = find_beacon(sl, g, corr_scale)
        confirms += 1
        if idx >= 0 and ratio < MIN_RATIO[0]:
            idx = -1
        if idx >= 0:
            meas = tk + max(0, off - 256) + idx - CORE_OFF_2NDREP
            resid = meas - pred
            if abs(resid) <= 64:
                return first, sense, w + 1, resid
            print("  confirm w=%d RESID=%d (k=%d ratio=%.2f) -> hunt restart"
                  % (w, resid, k, ratio))
            first = None
            sense = None
        elif confirms <= 5 or confirms % 20 == 0:
            print("  confirm w=%d attempt %d: no detect in slice (k=%d off=%d)"
                  % (w, confirms, k, off))
    return None, None, max_windows, None


def track(ue, anchor, sense, corr_scale, want, max_windows):
    name, g = sense
    resids, ks = [], []
    for w in range(max_windows):
        if len(resids) >= want or over_budget():
            break
        tk, c = ue.window()
        if tk is None:
            continue
        k = int(round((tk + 2048 - anchor) / FRAME))
        pred = anchor + k * FRAME
        off = pred - tk
        if not (0 <= off < len(c) - 700):
            continue
        sl = c[max(0, off - 256):off + 496 + 384]
        idx, rr = find_beacon(sl, g, corr_scale)
        if idx >= 0 and rr < MIN_RATIO[0]:
            idx = -1
        if idx >= 0:
            meas = tk + max(0, off - 256) + idx - CORE_OFF_2NDREP
            resids.append(meas - pred)
            ks.append(k)
    return np.array(resids, dtype=np.int64), np.array(ks, dtype=np.int64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bs-ip", default="168.6.244.21")
    ap.add_argument("--ue-ip", default="168.6.244.22")
    ap.add_argument("--core", default="beacon_core.bin")
    ap.add_argument("--gold", default="gold.bin")
    ap.add_argument("--matches", type=int, default=100)
    ap.add_argument("--rearms", type=int, default=2)
    ap.add_argument("--remakes", type=int, default=2)
    ap.add_argument("--corr-scale", type=float, default=10.0)
    ap.add_argument("--min-ratio", type=float, default=1e-2,
                    help="absolute detector-ratio floor: the real "
                         "beacon scores >=1e7 on this bench; noise-"
                         "window artifacts score ~500")
    ap.add_argument("--budget", type=float, default=600.0,
                    help="wall-clock budget (s); loops break gracefully so teardown + json always run")
    ap.add_argument("--tx-ch", type=int, default=1)
    ap.add_argument("--rx-ch", type=int, default=1)
    ap.add_argument("--out", default="two_node_beacon_arrival.json")
    args = ap.parse_args()
    DEADLINE[0] = time.time() + args.budget
    MIN_RATIO[0] = args.min_ratio

    core = np.fromfile(args.core, dtype=np.int16)
    cc = core[0::2].astype(np.float64) - 1j * core[1::2]  # conjugate
    pk = np.abs(cc).max()
    ram = np.zeros(2 * 4096, dtype=np.int16)
    ram[0:2 * len(cc):2] = np.round(cc.real / pk * 0.6 * 32767).astype(np.int16)
    ram[1:2 * len(cc):2] = np.round(cc.imag / pk * 0.6 * 32767).astype(np.int16)
    gold = np.fromfile(args.gold, dtype=np.complex64).astype(np.complex128)
    print("beacon core %d samp (RAM peak %d), gold %d taps"
          % (len(cc), int(np.abs(ram).max()), len(gold)))

    out = {"phases": []}
    bs = Bs(args.bs_ip, ram, args.tx_ch)
    ue = None
    rc = 0
    try:
        # Two sessions even single-board: TX-only BS session, RX-only UE
        # session -- map_scan's structure, the one that measures a STRONG
        # path. A shared session (TX setupStream with an in-session RX
        # stream) measured only crosstalk twice; see the ledger.
        bs.open_and_arm()
        print("BS armed, epoch=%d tx_freq_rb=%.1f" %
              (bs.epoch, bs.dev.getFrequency(SOAPY_SDR_TX, args.tx_ch)))
        if not bs.liveness():
            print("FAIL: beacon not playing (liveness); aborting")
            return 4
        ue = Ue(args.ue_ip, args.rx_ch)
        peek = 0.0
        for _ in range(8):
            srp = ue.dev.readStream(ue.rxs, [ue.buf], 12288, timeoutUs=1000000)
            if srp.ret > 0:
                wp = ue.buf[:2 * srp.ret].astype(np.float64)
                peek = max(peek, float(np.sqrt(np.mean(wp * wp))))
        print("UE peek (8 undrained reads): max rms=%.1f  %s"
              % (peek, "STRONG" if peek > 50 else "weak/noise"))

        anchor, sense, nwin, resid0 = acquire(ue, gold, args.corr_scale)
        if anchor is None:
            print("FAIL: no acquisition in %d windows" % nwin)
            return 2
        print("M1 lock: anchor=%d sense=%s windows=%d confirm_resid=%d"
              % (anchor, sense[0], nwin, resid0))

        resids, ks = track(ue, anchor, sense, args.corr_scale,
                           args.matches, args.matches * 25)
        if len(resids) < args.matches // 2:
            print("FAIL: only %d/%d tracked matches" % (len(resids),
                                                        args.matches))
            rc = 3
        drift = 0.0
        if len(ks) > 2:
            drift = float(np.polyfit(ks - ks[0], resids, 1)[0])
        print("M2 track: %d matches  resid mean=%.2f std=%.2f min=%d max=%d "
              "drift=%.6f samp/frame  span=%d frames"
              % (len(resids), resids.mean(), resids.std(), resids.min(),
                 resids.max(), drift, (ks.max() - ks.min()) if len(ks) else 0))
        out["phases"].append(dict(phase="baseline", epoch=bs.epoch,
                                  anchor=int(anchor), sense=sense[0],
                                  resid_mean=float(resids.mean()),
                                  resid_std=float(resids.std()),
                                  drift=drift, n=len(resids)))
        base_epoch, base_anchor = bs.epoch, anchor

        def remeasure(tag):
            a2, s2, nw, r0 = acquire(ue, gold, args.corr_scale)  # noqa: B023 (late binding intended)
            if a2 is None:
                print("  %s: reacquire FAILED" % tag)
                return None
            d_meas = (a2 - base_anchor) % FRAME
            d_pred = (bs.epoch - base_epoch) % FRAME
            err = (d_meas - d_pred + FRAME // 2) % FRAME - FRAME // 2
            print("  %s: epoch=%d anchor=%d windows=%d "
                  "phase_shift meas=%d pred=%d err=%d samples"
                  % (tag, bs.epoch, a2, nw, d_meas, d_pred, err))
            out["phases"].append(dict(phase=tag, epoch=bs.epoch,
                                      anchor=int(a2), meas=int(d_meas),
                                      pred=int(d_pred), err=int(err)))
            return err

        print("M3 re-arms (framer determinism, tiles untouched):")
        for i in range(args.rearms):
            bs.rearm()
            remeasure("rearm%d" % (i + 1))

        print("M4 re-makes (tile bring-up variability, the no-MTS probe):")
        same_board = (args.bs_ip == args.ue_ip)
        for i in range(args.remakes):
            if same_board:
                # construction resets the tiles under a live stream on the
                # same board: rebuild the UE session per re-make. Both the
                # epoch and the new UE stamps ride the same board clock, so
                # an origin reset cancels in the prediction.
                ue.close()
            bs.close()
            time.sleep(2)
            bs.open_and_arm()
            print("  remake%d: tx_freq_rb=%.1f" %
                  (i + 1, bs.dev.getFrequency(SOAPY_SDR_TX, args.tx_ch)))
            if same_board:
                ue = Ue(args.ue_ip, args.rx_ch)
            remeasure("remake%d" % (i + 1))
    finally:
        if ue is not None:
            ue.close()
        bs.close()
        with open(args.out, "w") as f:
            json.dump(out, f, indent=1, sort_keys=True)
        print("wrote %s" % args.out)
    return rc


if __name__ == "__main__":
    sys.exit(main())
