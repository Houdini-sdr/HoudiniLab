#!/usr/bin/env python3
"""Synthetic CSI/constellation/ADC feed, so the dashboard can be seen with no radios.

Sends the same three datagram kinds `sounder --view` sends, at the same rates, to a
running ``csi_server.py``. Use it to look at the dashboard, to check a change to the
page, or to reproduce a display bug without booting the rig:

    python3 csi_server.py &            # terminal 1, or a separate window
    python3 fake_feed.py               # terminal 2

Then open the dashboard as usual. Two antennas appear within a second.

Nothing here is a model of the radio. The channel is an arbitrary smooth response
with a couple of nulls; it exists to give every panel something recognisable to draw.
Do not read it as expected hardware behaviour.

Failure modes worth reproducing on purpose:

    python3 fake_feed.py --clip           # ADC hard against the rail, clipping badge
    python3 fake_feed.py --reps 1         # slot with one pilot symbol, quality panel
                                          #   reports "not measurable" instead of 1.0
    python3 fake_feed.py --noise 0.5      # low repeat coherence, quality trace drops
    python3 fake_feed.py --legacy         # emit CSI1, as an un-rebuilt sounder would
    python3 fake_feed.py --antennas 4     # more cards
    python3 fake_feed.py --stall-after 60 # stop sending, to watch the stale badges
"""
import argparse
import math
import random
import socket
import struct
import time

MAGIC_CSI = 0x43534931
MAGIC_CSI2 = 0x43534932
MAGIC_CNS = 0x434E5331
MAGIC_ADC = 0x41444331
MAGIC_ADC2 = 0x41444332
ADC_FS = 32767
ADC_COLS = 250


def channel(nsc, ant, frame):
    """A smooth response with two nulls, drifting slowly so the waterfall moves."""
    h = []
    for k in range(nsc):
        if k == 0 or k >= nsc - 4:        # DC null + guard band, as the sounder sends
            h.append(0j)
            continue
        f = k / nsc
        mag = (0.55
               + 0.30 * math.sin(2 * math.pi * (f + frame * 0.004 + 0.2 * ant))
               + 0.12 * math.sin(6 * math.pi * f))
        mag = max(0.02, mag)
        phase = 2 * math.pi * f * (3.0 + 0.3 * ant) + 0.4 * math.sin(4 * math.pi * f)
        h.append(complex(mag * math.cos(phase), mag * math.sin(phase)))
    return h


def send_csi(sock, dest, frame, ant, h, rate, reps, noise, legacy):
    nsc = len(h)
    if legacy:
        head = struct.pack("<IIIIf", MAGIC_CSI, frame, ant, nsc, rate)
        body = b"".join(struct.pack("<ff", z.real, z.imag) for z in h)
        sock.sendto(head + body, dest)
        return
    head = struct.pack("<IIIIfI", MAGIC_CSI2, frame, ant, nsc, rate, reps)
    body = b"".join(struct.pack("<ff", z.real, z.imag) for z in h)
    # Coherence the way the sounder derives it: weak subcarriers lose to the noise
    # first, so the quality trace tracks the nulls in the magnitude panel.
    qual = []
    for z in h:
        m = abs(z)
        if m == 0.0:
            qual.append(0.0)          # unused tone; the backend turns this into a gap
        elif reps < 2:
            # What the C++ actually emits here: the coherence of a single term is
            # exactly 1.0 however bad the noise. Sending the honest-looking 0.0
            # instead would make this feed useless for testing the case.
            qual.append(1.0)
        else:
            snr = (m * m) / max(1e-9, noise * noise)
            qual.append(min(1.0, snr / (snr + 1.0 / max(1, reps - 1))))
    sock.sendto(head + body + struct.pack("<%df" % nsc, *qual), dest)


def send_cns(sock, dest, frame, ant, mod, evm):
    pts = []
    lv = int(round(math.sqrt(2 ** mod)))
    levels = [-(lv - 1) + 2 * i for i in range(lv)]
    power = sum(a * a + b * b for a in levels for b in levels) / (lv * lv)
    nrm = math.sqrt(power)
    for _ in range(240):
        a = random.choice(levels) / nrm
        b = random.choice(levels) / nrm
        pts += [a + random.gauss(0, evm), b + random.gauss(0, evm)]
    head = struct.pack("<IIIII", MAGIC_CNS, frame, ant, len(pts) // 2, mod)
    sock.sendto(head + struct.pack("<%df" % len(pts), *pts), dest)


def send_adc(sock, dest, frame, ant, samps, rate, clip, legacy=False):
    """A burst inside an otherwise quiet slot, optionally driven into the rail.

    Emits ADC2 (pilot-slot envelope + an all-slot clip ledger) unless `legacy`,
    which emits ADC1 as a sounder built before the split does.
    """
    amp = ADC_FS if clip else int(0.45 * ADC_FS)
    env, peak, clipped = [], 0, 0
    per_col = max(1, samps // ADC_COLS)
    for c in range(ADC_COLS):
        # the burst occupies the middle half of the slot
        active = 0.25 * ADC_COLS <= c < 0.75 * ADC_COLS
        a = amp if active else int(0.02 * ADC_FS)
        env.append((-a, a, -int(a * 0.9), int(a * 0.9)))
        peak = max(peak, a)
        if clip and active:
            clipped += per_col            # every sample in the column is at the rail
    body = b"".join(struct.pack("<4h", *e) for e in env)
    if legacy:
        head = struct.pack("<IIIIIfII", MAGIC_ADC, frame, ant, ADC_COLS, samps, rate,
                           min(peak, ADC_FS), clipped)
    else:
        # the ledger reports a louder, undrawn slot, which is the case the split exists
        # for: a beacon slot can clip while the drawn pilot slot looks healthy
        head = struct.pack("<IIIIIfIIIII", MAGIC_ADC2, frame, ant, ADC_COLS, samps, rate,
                           min(peak, ADC_FS), clipped, 16,
                           min(int(peak * 1.7), ADC_FS), clipped)
    sock.sendto(head + body, dest)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="dashboard UDP host")
    ap.add_argument("--port", type=int, default=9999, help="dashboard UDP port")
    ap.add_argument("--antennas", type=int, default=2)
    ap.add_argument("--subcarriers", type=int, default=64)
    ap.add_argument("--rate", type=float, default=30.72e6, help="sample rate, Hz")
    ap.add_argument("--fps", type=float, default=30.0, help="frames per second")
    ap.add_argument("--frames", type=int, default=0, help="0 = run until interrupted")
    ap.add_argument("--reps", type=int, default=6,
                    help="pilot symbols averaged per slot; below 2 the quality panel "
                         "reports that it cannot measure")
    ap.add_argument("--noise", type=float, default=0.05,
                    help="per-symbol noise, drives the repeat coherence down")
    ap.add_argument("--evm", type=float, default=0.08, help="constellation cloud size")
    ap.add_argument("--mod", type=int, default=2, help="bits/symbol: 2 QPSK, 4 16QAM")
    ap.add_argument("--samps", type=int, default=4096, help="samples per slot")
    ap.add_argument("--clip", action="store_true",
                    help="drive the ADC into the rail, to see the clipping badge")
    ap.add_argument("--legacy", action="store_true",
                    help="send CSI1, as a sounder built before the quality panel does")
    ap.add_argument("--stall-after", type=int, default=0, metavar="N",
                    help="stop sending after N frames, to watch the stale badges")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    print("[fake] feeding %s:%d, %d antenna(s), %.0f fps%s"
          % (args.host, args.port, args.antennas, args.fps,
             ", CSI1 legacy" if args.legacy else ""), flush=True)
    frame = 0
    try:
        while args.frames == 0 or frame < args.frames:
            if args.stall_after and frame >= args.stall_after:
                print("[fake] stalled after %d frames, cards should go stale"
                      % args.stall_after, flush=True)
                while True:
                    time.sleep(1.0)
            for ant in range(args.antennas):
                h = channel(args.subcarriers, ant, frame)
                send_csi(sock, dest, frame, ant, h, args.rate, args.reps,
                         args.noise, args.legacy)
                send_cns(sock, dest, frame, ant, args.mod, args.evm)
                send_adc(sock, dest, frame, ant, args.samps, args.rate, args.clip,
                         args.legacy)
            frame += 1
            time.sleep(1.0 / args.fps)
    except KeyboardInterrupt:
        pass
    print("[fake] sent %d frames" % frame, flush=True)


if __name__ == "__main__":
    main()
