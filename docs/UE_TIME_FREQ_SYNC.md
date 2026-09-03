# UE time and frequency sync on free-running clocks

What the 2026-09-01 campaign established, what is still open, and why the
priorities are ordered the way they are. Evidence rows live in
`CC/Sounder/DEMO_VERIFICATION.md` section 8; tracker rows are AP-31, AP-33,
AP-34, AP-37..AP-44.

**One-line state.** The UE holds the BS clock to a couple of samples on
independent internal references, verified independently from the BS side. There
is no frequency correction on the data path at all.

## 1. The clocks, measured

`clock_ref` is a device setting (`houdini-provision --set-clock-ref`, LMK
register 0x147), so switching a board to its onboard reference is the only
lever on this bench that produces real sample-clock drift.

| leg | `.21` | `.22` | eps = (f_BS - f_UE)/f_UE |
| --- | --- | --- | --- |
| A control | external | external | **0.0000 ppm** (slope 0.0000 samp/s, 0.0 jitter) |
| B | external | internal | -7.2019 ppm |
| C | internal | external | -1.3305 ppm |
| D target | internal | internal | **-8.5197 ppm** |

Legs B and C PREDICT leg D at -8.532 ppm. The ladder closes to **0.02 ppm**
across three instruments, one of which (`hwtime_rate_probe.py`, a
getHardwareTime ratio) touches no RF at all. So the onboard references are
+7.20 ppm (`.22`) and -1.33 ppm (`.21`), ordinary crystal numbers and 100x
smaller than the struck 894 ppm figure.

At leg D: **-4.26 kHz of carrier offset and 1.047 samples of timing drift per
frame.** Acquisition is untouched (the tolerance is ~100 kHz); the timing is
what breaks.

**Stability.** Allan deviation over 600 s / 36904 detections is below 0.04 ppm
out to tau = 10 s, but that region is MEASUREMENT-floor limited (flat ~48
samples of apparent drift, ADEV falling as 1/tau). Real wander appears past
tau ~ 50 s. Dominating all of it: eps moved **-8.5197 -> -8.2895 ppm across one
session, 0.23 ppm**. Cadence choices therefore carry a safety factor rather than
being tuned, and this bench is WIRED; OTA will add more.

## 2. What the timing loop does, and how it is verified

The UE estimates the BS clock in its own sample units as two states,
`(ref, period)`, and derives every prediction from it. Three things were wrong
before and are worth keeping straight:

1. The frame period was ASSUMED exactly `samps_per_frame`. True only on a
   shared reference.
2. The acquisition confirm was already MEASURING the rate (`resid/k` over k real
   frames) and discarding it. Seeding from it is what makes the loop close: the
   tracker otherwise gets ~2.8 attempt opportunities inside the 978 ms the drift
   needs to spend the whole tolerance, and collects zero observations.
3. The pilot burst ladder stepped by the NOMINAL period regardless of the
   tracker, so the pilot walked even with the UE locked. Proven by tracing the
   commanded burst times: 100.0% exact multiples of 122880 before, 99.9% exact
   multiples of the tracked period after.

**Verification, and why it counts.** `pilot_grid_off` is a BS-side measurement
of where the UE's pilot actually landed against the BS's own grid. It knows
nothing about the UE's belief, so it is an independent truth check rather than
the UE grading itself.

| | control (alpha=0 beta=0) | tracker, 3 runs |
| --- | --- | --- |
| UE sync residual | n/a | **sd 0.63-0.70, max \|resid\| 2-3** |
| escalations | 2 | **0 / 0 / 0** |
| BS `pilot_grid_off` | swept the whole frame | median **-37**, p5/p95 spread 3 samples |

The timing loop also corrects the SAMPLE-CLOCK offset, which is the dominant
term. Only the carrier phase is left over.

## 3. What is NOT corrected

There is no derotator. `kEnableCfo` is a compile-time `false`, the legacy
`estimateCFO` call behind it is dead, and the `TODO: measure CFO from the first
beacon and apply here` is still a TODO.

**The predicted consequence, NOT yet observed (AP-37).** H is estimated in the P
slot and applied in the U slot, 8192 samples = 66.7 us apart. At -4260 Hz that
is **102 degrees of common rotation**. Nothing removes it: the blind 4th-power
search corrects a phase RAMP (timing r) and its score is magnitude-based, so a
frequency-flat rotation passes through. Neither can the CNS quality counter see
it, because `score = |mean(u^4)|` is rotation-invariant by construction. The
0.955-0.967 measured on internal clocks says the constellation is TIGHT, not
correctly oriented, and every EVM row in the ledger predates internal clocks.

Check this before building for it.

## 4. What the CFO estimator does wrong, and the fix

`estimateCFO` is precise but not accurate: on a link where the arrival ramp
measures eps = 0 exactly it reads +353 Hz, and across the four legs its error
ranged +280 to +1753 Hz with no consistent slope.

Root-caused offline. The bias is **detector index misalignment, and it is
asymmetric**:

| delta | estimate at true CFO = 0 | why |
| --- | --- | --- |
| -6 .. -1 | +226 to +2817 Hz | window 1 dips into the STS; STS x gold cross-correlation injects phase |
| 0 | 0.0 Hz | aligned |
| +1 .. +6 | **exactly 0.0 Hz** | window 2 runs into the beacon's trailing zeros, so those terms multiply by zero |

`find_beacon` uses an EARLIEST-CROSSING rule, which biases `sync_index` early by
construction. That is why every observed disagreement was positive. Simulating
delta uniform in [-6,0] reproduces the field data (mean +1487.5 Hz, sd 1215); a
**+8 sample guard** gives mean -31.8 Hz, matching a full peak-refinement search,
and is monotonically safe because delta can only become more positive.

It is NOT a scale error: at delta = 0 the estimator is exact at every CFO
tested, so the earlier "~303 MHz effective carrier" reading is withdrawn.
Residual sd ~405 Hz is thermal noise at lag 128, which is AP-34(b)'s problem.

**Meanwhile, take the CFO from the TRACKED CLOCK.** The sample clock and the
RFDC NCO both derive from the one LMK PLL1 reference, so the fraction the
tracker measures off timing is the fraction the carrier carries:
`eps = samps_per_frame/period - 1`, `cfo = eps * freq`. Live, same frames: the
tracked value read a constant **-4127.0 Hz** while the beacon estimator swung
**1300 Hz**. This is easier for us than for the distributed-MIMO literature,
which has to measure phase directly.

## 5. How this compares to WiFi, LTE, and the distributed-MIMO work

- **802.11**, and our own `MATLAB/rl_ofdm_siso.m`, use TWO tiers: a bulk CFO
  estimate from the repeated LTS (lag 64) applied as a waveform derotation, and
  **per-symbol common-phase tracking from the 4 pilot subcarriers**. Note the
  reference ships with `APPLY_CFO_CORRECTION = 0` and
  `DO_APPLY_PHASE_ERR_CORRECTION = 1`: tier 2 alone carries it.
- **LTE / srsRAN** track frequency error from reference signals into a frequency
  rotator, and the UE also applies a proportional correction to its UPLINK
  frequency. Their docs name our exact regime: off-the-shelf front ends give
  >1 kHz offsets against GPS-disciplined base stations.
- **AirSync** (distributed MIMO, independent oscillators) locks nodes from a
  common reference broadcast over the air "in conjunction with a Kalman filter
  which closely tracks the phase drift" and predicts forward a few frames. That
  is structurally what we built, with a Kalman where we have alpha-beta.
- **AirShare** uses a dedicated two-tone emitter specifically for robustness to
  temperature and supply-voltage drift, which is the 0.23 ppm term above.
- A 2025 256-antenna distributed MIMO testbed still distributes a 10 MHz
  reference over OctoClocks. Wiring the clock remains the state of the art for a
  large testbed; free-running with OTA tracking is the harder road.

**The lesson common to all of them:** everyone running free-running nodes uses a
PREDICTIVE FILTER over a periodic over-the-air reference, and nobody relies on a
single-shot preamble estimate. Both standards also put the phase reference in or
adjacent to the data symbol, rather than extrapolating a channel estimate 66.7 us
in time as we currently do.

## 6. Cost model (measured on the rig, and it drives several decisions)

- `radioRx` = **855 us fixed + 0.0037 us/sample**. The cost is PER CALL, not per
  byte: marginal capability is 273 Msample/s against a 122.88 MSPS stream. This
  is why coalescing 30 reads per frame into one took the iteration from 27.5 ms
  to 2.4 ms.
- Correlator = **0.0260 us/sample, 38.5 MSPS sustained**. A full frame is
  3.18 ms (fine, it is one-shot at acquisition); the tracking slice is 0.05 ms;
  continuous full-rate correlation is impossible and always was, which argues
  for the scheduled-read design rather than against it.
- Fronthaul carries **495.3 MB/s** while the app keeps **16.4**. 97% waste, and
  separately NOT the reason reads are slow.
- Acquisition window: expected time is `F*a_read/W + F*(b_read+b_corr)`, whose
  second term is independent of W, so cost falls with W until the hit
  probability saturates. The optimum is **exactly one frame plus the beacon
  core**, at any sample rate.

## 7. Terminology, because it caused confusion

The often-quoted "1.4% beacon hit rate" is wrong. **The hit rate is 100%**: 485
searches produced 486 detections, and 543 produced 543. The ~1% is the fraction
of loop ITERATIONS on which a search is attempted at all. Of those detections,
34-38% pass the SNR floor, and the distribution is sharply bimodal (median
48.6 dB accepted, 11.2 dB rejected) because the low mode is the beacon
straddling the slice edge, where the arrival time would be biased. The floor is
doing its job, not wasting good detections.

## 8. The clock actuator, and what the software layer is now for

The software lane shipped `CLOCK_ADJ` (SH-341): the LMK holdover DAC as a
runtime setting, an actuator only under `clock_ref = calibrated`. Both nodes now
hold their VCXO at a code tracked to the shared 10 MHz, free-running, re-entered
at every boot. `.21` cal_dac 408, `.22` cal_dac 404.

**This is the point the campaign was walking toward.** Timing drift and carrier
offset were never two problems: they are one reference error seen two ways,
which is exactly why the timing tracker reports the carrier offset for free. An
actuator at that reference nulls both at once.

### Measured, by us

| quantity | value | note |
| --- | --- | --- |
| actuator gain | **-0.1251 ppm/count** | protocol says 0.129 (3% off), bench doc 0.087 (44% off, a stated lower bound) |
| linearity | 0.059 ppm rms over ~5 ppm | 6-point sweep |
| hold wander | 0.19 ppm / 7 min (~1.5 counts) | matches the lane's "a count in ten minutes" |
| loop convergence | **0.57 -> 0.06 ppm in 2 pushes, 3/3 runs** | same landing code each time, no overshoot |

Using the bench doc's 0.087 would have made the loop gain 1.44x too large. That
is why the gain was measured before the loop was closed.

**Sign, because it is the easiest thing to invert:** eps = (f_BS - f_UE)/f_UE
and we steer the UE, so f_UE sits in the numerator AND the denominator and
raising it LOWERS eps. A negative d(eps)/d(count) therefore means higher code =
FASTER UE. My own calibration script printed this backwards once; an inverted
gain is positive feedback.

### The software tracker is not superseded, its role changed

Outer loop = clock steer (slow, seconds to minutes). Inner loop = the grid
tracker. Five reasons the inner loop stays:

1. **It is the sensor.** The steer's error signal is eps and nothing else
   measures it.
2. **Steering fixes RATE, never OFFSET.** Propagation delay and RF-chain latency
   are a static time offset no frequency correction touches. LTE keeps timing
   advance on frequency-locked networks for exactly this reason.
3. **Bandwidth.** The steer is deliberately slow; re-acquisition and glitch
   recovery are inner-loop work.
4. **Quantum and range.** 0.1251 ppm/count leaves +-0.065 ppm for software, and a
   node not in `calibrated` mode refuses the actuator outright.
5. **Doppler.** The steer must NOT chase motion, so the inner loop has to carry
   the fast-varying part. That division of labour IS the invariance.

### Doppler invariance requires a two-way measurement

Correcting a claim made earlier in this campaign: the timing channel is **not**
Doppler-immune. A range rate changes the propagation delay continuously, which
is an apparent time scaling indistinguishable from a clock rate offset -- a
Doppler shift IS a time dilation. On a one-way link the two are degenerate at
any lag, and no estimator separates them.

The separation is the standard two-way result: the clock term flips sign between
directions, the range-rate term does not.

    downlink = +eps - rdot/c        uplink = -eps - rdot/c
    difference -> clock             sum -> Doppler

We already produce both halves (the UE's arrival ramp and the BS's
`pilot_grid_off`), and both threads live in one process, so the reporting path
on this bench is a shared value rather than a protocol. **One subtlety that
would bite an implementer:** our uplink half is not independent, because the UE
transmits on its TRACKED grid -- so `pilot_grid_off`'s slope is the tracker's
residual, not the raw uplink rate, and with a converged tracker it goes to zero.
The UE knows exactly what grid rate it applied, so adding it back de-embeds the
correction and restores independence.


## 10. The cadence and the gate, as measured [2026-09-02]

Sections 1 to 9 were written when the client resynced every 260 ms and the
alive/moved gate admitted +-1024 samples. Both defaults changed on 2026-09-02,
and the reasoning is worth carrying because it is the same reasoning that will
apply next time.

**Neither old value came from a measurement of the thing it governed.** The
cadence was derived from an ASSUMED 1.0 ppm post-tracking residual. The gate's
+-1024 dated from the era before targeted resync, when the detector could anchor
hundreds of samples early.

**What the measurements say.** Three 300 s captures, with the binning artifact
of section 4 fixed, give a proper Allan deviation bathtub: the minimum sits at
tau = 2 s, drift there is 0.46 to 0.58 samples, and at tau = 20 s it is 18 to 23
samples against a 32-sample budget. That implies an effective residual of 0.002
ppm at 2 s and 0.008 at 20 s, so the 1.0 ppm assumption was 120 to 500 times
pessimistic. Separately, the detection residual under targeted resync measures
-2 to +3 samples across about twenty runs, worst case 6, so the +-1024 gate was
roughly 170 times the worst case.

**What was changed, and what was deliberately left.** The cadence went to 2.6 s
(0.1 ppm assumed), keeping 12 to 50 times margin on the measured rate. The full
measured margin would be 26 s, and that was NOT taken: 20 s is where the Allan
deviation data ends and beyond it the number would be an extrapolation rather
than a measurement. The gate went to 2.0 us, 246 samples, which is 41 times the
worst observed residual and is the only tighter value that has actually been run
on silicon.

**A tighter gate finds MORE beacons, not fewer.** This is the counterintuitive
part and it is worth stating plainly. The targeted search needs a slice of
kLead + kTail samples inside the read, and both scale with the tolerance. So
tightening the tolerance shrinks the slice and WIDENS the window of read phases
that can host an attempt, from 42 % of a slot to 80 %. Measured: 153 accepted
detections in 60 s against the baseline's 91.

**Consequence for anyone reading a gate result.** The verification campaign in
DEMO_VERIFICATION section 8 was run against the OLD defaults. A 10x cadence
means about 10x fewer detections per run, so accept counts and residual standard
deviations from those runs are not comparable to new ones, and the gate criteria
have to be restated before the next gate rather than after it.

## 9. What to keep, and what not to

- **KEEP the software grid tracker** as sensor and inner loop, per above.
- **KEEP the per-symbol pilot phase correction**, without overselling it:
  measured 13 vs 15 low frames of 2048 at zero offset, 31 vs 38 at 2000 Hz.
  Consistent, no regression, but small -- because steering removed the cause and
  left ~0.7 deg to correct. It does NOT rescue large offsets: at 20 kHz it hurts,
  because that smears H itself and a per-symbol scalar cannot repair a
  per-subcarrier error.
- **KEEP the beacon phase estimator** now that its bias is fixed (-724.7 Hz mean
  before, within +-69 after, three runs). Not as a clock source -- the timing
  channel is 20x better -- but as the only channel that can ever see Doppler.
- **DROP the longer-lag estimator at this priority.** It needs detection pairs
  <= ~9 frames apart where ours arrive ~260 apart, so it is a scheduling change;
  it cannot bootstrap itself (the lag-128 estimate cannot unwrap even one frame);
  and it buys 0.05 Hz where the timing channel gives 18 and the actuator quantum
  is 64. Doppler separation, the reason to keep a carrier channel at all, is
  two-way.
- **DROP UE TX frequency pre-compensation** except for non-steerable nodes:
  steering the reference moves the TX NCO with it.
- **RETUNE rather than remove** the escalation thresholds, the scatter gate, the
  alpha-beta gains and the resync cadence. All were derived against 8.5 ppm; the
  steered regime is two orders quieter.

## 11. The NR form, and what a repeating beacon can and cannot give [2026-09-03]

Written for AP-66 and the question that came with it: is an NR-style time and
frequency sync a big change here, and does phase need downlink pilots or can
the beacon carry it. The short answers: the NR acquisition architecture costs
five files and about 150 lines against the beacon we already send (section
11.2); the beacon can carry phase to roughly 5 degrees across a 1 ms frame,
which is enough for QPSK and 16-QAM and not for anything better or for
anything coherent across nodes (section 11.4).

### 11.1 What NR actually does (TS 38.211 7.4.2 and 7.4.3, TS 38.213 4.1)

The SS/PBCH block is four OFDM symbols over 240 subcarriers: PSS in symbol 0,
PBCH in 1 and 3, SSS in the middle of symbol 2 with PBCH either side. The PSS
is a length-127 BPSK m-sequence, x(i+7) = x(i+4) + x(i) mod 2, with three
cyclic shifts carrying N_ID2. The SSS is the product of two length-127
m-sequences with 336 x 3 shift pairs carrying the cell identity. The PBCH
demodulation reference is scrambled by the cell identity and the SSB index.

The UE procedure, as the two reference implementations do it (MATLAB's NR cell
search example; srsRAN's `ssb.c`):

1. **Time-domain matched filter on the PSS.** The received samples are
   correlated against the time-domain PSS waveform for each of the three
   candidates. srsRAN does it as an FFT-domain product with a correlation size
   at least 8x the symbol, applies NO threshold, and takes the maximum; MATLAB
   repeats the search across carrier-offset hypotheses spaced half a
   subcarrier apart. The peak gives symbol timing and N_ID2.
2. **Fine frequency.** MATLAB correlates each symbol's cyclic prefix with its
   tail. srsRAN takes the phase between the PSS and SSS least-squares channel
   estimates one symbol apart: `cfo = arg(corr_sss * conj(corr_pss)) / (2 pi
   dt)`. Both are the repetition-phase estimator this repo's stages 1 and 2
   use, applied to two known symbols instead of two copies of one.
3. **SSS in the frequency domain,** at the timing the PSS gave, against the 336
   candidates. This is ALSO the confirmation that the PSS peak was real: NR's
   false-lock guard is a second sequence, not a repeat structure.
4. **PBCH DMRS gives the SSB index,** which tells the UE where in the frame the
   block sat; the decoded MIB gives the frame number. Our frame puts one beacon
   at a fixed position, so this stage has nothing to tell us.
5. **Delay refinement** from the phase slope of the channel estimate across
   subcarriers (srsRAN), which is sub-sample timing for free once the FFT has
   been taken.
6. **Afterwards,** fine time and frequency tracking from the TRS (a periodic
   CSI-RS burst, two symbols in each of two slots, every 10 to 80 ms; TS 38.214
   5.1.6.1.1); channel and phase per slot from the DMRS inside the data; the
   per-symbol common phase error from the PTRS (TS 38.211 7.4.1.2), which is
   dense in time and sparse in frequency because phase noise rotates every
   subcarrier alike.

Four facts to carry from this. The acquisition detector is a plain matched
filter on a sequence that does NOT repeat, so there is no lag product and no
repeat check. The false-lock guard is a second decoded sequence. Frequency comes
from two known symbols at a known spacing. Phase is never carried from the SSB
into the data; it is re-measured inside every slot.

### 11.2 How that maps onto our beacon, and what it cost to do

The `nr` shape already transmitted the standard's PSS (127 tones in a 128-point
IFFT, which at 122.88 MSPS is the 960 kHz numerology, DC nulled, no cyclic
prefix), then a 16-sample guard and two copies of a 64-tone TRS symbol built on
the 38.211 Gold sequence. Every measurement of it through 2026-09-02 correlated
on the TRS pair through the lag product, so it measured OUR detector on an NR
waveform and found it 5 dB short of legacy (DEMO_VERIFICATION 8.134). The
"NR-style detector loses" row (8.144) dropped the lag product but kept the
repeated replica, and measured the rep1/rep2 ambiguity of a matched filter
against a symbol that appears twice.

`nr_pss` (2026-09-03) is the transmit-identical control: the same core, the PSS
as the replica, the plain matched filter forced by the replica's single copy,
the first-path pick unchanged. Offline it holds every prediction that was
written before it ran: exact at all seven levels and eight noise draws, one
`corr_scale` across a 64x level sweep, processing gain 37.8 dB against legacy's
37.7, first-path unbiased on all six multipath channels at 8.5 ppm. The
architecture was never the problem; the replica was. Silicon is 8z.

Scope, measured by doing it rather than estimated: `beacon_shapes.h` (the
shape, plus `replica_off`/`replica_reps` so the beacon end is derived rather
than assumed), `config.h`/`config.cc` (two accessors), `receiver.cc` (the
detector form follows the replica; the 144-sample replica tail is added in the
one place both search paths pass through), the geometry test, the dumper and
the probe. About 150 lines. The frame, the slot, the replay RAM, the tracker,
the CFO estimator, the SNR guard and the beacon-end convention are untouched.

What is still NOT NR about it, in the order it would matter over the air:

- no SSS, so the false-lock guard stays the in-window SNR floor plus the grid
  residual (a decoded second sequence would be stronger and cheaper than an
  SNR window on a fading channel);
- no cyclic prefix on the PSS, so a multipath echo longer than a sample or two
  contaminates the symbol instead of rotating it;
- the PSS fills the whole band instead of 127 of 240 subcarriers, which is fine
  here and would not be in a shared band;
- frequency comes from the TRS repetition, not from the PSS-to-SSS phase.

An SSB-lite that adds the SSS with NR cyclic-prefix lengths at the 240 kHz
numerology (512-point symbols, 36-sample prefix, four symbols = 2192 samples,
inside the 4096-sample replay RAM) closes all four. It is about a day: SSS
generation is thirty lines against the same 38.211 recurrences, the
frequency-domain SSS check and the PSS-to-SSS CFO estimator are one FFT each,
and nothing in the driver or the gateware moves. That is the item to take up if
the OTA target wants NR-shaped acquisition; it is not needed to answer AP-66.

Why the proof of concept did not link srsRAN or OpenAirInterface: both assume
the full 240-subcarrier block with PBCH, and at the 30 kHz numerology one block
is 4 x 4384 = 17536 samples, over four times the replay RAM. At 240 kHz it
fits, but the library's PBCH and MIB machinery exists to tell a UE WHERE in the
frame it is, which our fixed beacon position already settles, and it would pull
a large dependency into the sounder for two sequences that
`include/beacon_shapes.h` already generates from the standard's own
definitions.

### 11.3 What the literature does with a repeating beacon

The classical estimators, all of which this repo already uses: timing from the
matched-filter peak; frequency from the phase of the repetition correlation,
coarse to fine by lag (Moose 1994, Schmidl and Cox 1997, Morelli and Mengali
1999 for L repeats); phase from the complex value of the matched-filter peak,
which is the carrier phase at that instant.

The distributed-MIMO work is the literature that runs independent oscillators
from a periodic over-the-air reference, and it is consistent to a fault:

- **AirSync** (Balan et al., 2013): the master broadcasts pilot tones
  continuously in a reserved part of the band; each secondary tracks the
  per-subcarrier phase drift and PREDICTS it a few OFDM symbols ahead with a
  linear or Kalman predictor to cover its own transmit pipeline latency.
  Implemented on WARP FPGAs; carrier phase coherence within a few degrees
  after correction.
- **MegaMIMO** (Rahul et al., 2012): in band, per packet; secondaries measure
  the lead node's reference in every packet and rotate their transmission by
  the accumulated phase.
- **BeamSync** (2023): the sync signal is beamformed in the dominant direction
  of the inter-node channel; a nonlinear least-squares phase estimator and a
  simpler one that matches it at high SNR; a frequency estimator; oscillator
  phase modelled as a discrete Wiener process, variance 4 pi^2 fc^2 c_vco Ts.
  Simulation only.
- **Merlo et al.** (2025, arXiv 2506.07267): two-way time transfer with pulsed
  two-tone waveforms (20 MHz tone separation, 1.5 us pulses, an 11.5 us epoch,
  resynchronised about every 40 ms) on X310 radios at 200 MSa/s: 60 to 70 ps
  time and phase precision, 3.73 ppb frequency RMSE, median coherent gain
  above 99 percent. Their stated rule for the reference interval: the update
  rate must sit well above the frequencies where the phase-noise and vibration
  spectrum has its power.
- **OTA phase calibration inside the TDD flow** (2025, arXiv 2509.03722): each
  node's oscillator phase is a Wiener random walk with variance
  4 pi^2 10^10 S(100 kHz) / fs, tracked pairwise with a Kalman filter; for
  S(100 kHz) = -100 dBc/Hz the beamforming loss becomes significant once the
  calibration interval passes 10 to 20 ms.

The common shape: a periodic reference; a two-state estimator (phase and
frequency, or offset and rate) with a random-walk process model; prediction
between references; and a reference interval set by the phase-noise spectrum
rather than by the estimator's own noise. Our timing tracker (AP-31, AP-41)
already has this shape. The phase side does not exist yet.

### 11.4 Do we need downlink pilots for phase? The arithmetic, with our numbers

What the beacon can carry. The frame-to-frame beacon phase IS coherent
(AP-34(b): circular resultant 0.99, circular sd 0.10 to 0.13 rad at the 1 ms
frame spacing over 160 pairs, at the calibrated clock state). A two-state phase
and frequency tracker on the beacon would therefore predict the carrier phase
across a frame with an innovation of at most about 7 degrees per ms, and that
figure includes the estimator's own noise. The frequency term alone is
2 pi df T, which at the 0.02 ppm the tracked clock agrees with the timing
channel (10 Hz at 500 MHz) is 3.6 degrees at the end of the frame. Mid-frame,
call it 4 to 5 degrees rms.

What that buys: QPSK and 16-QAM downlink demodulation with no pilot in the
slot (a phase error floor near -20 dB EVM), and a per-frame phase reference
good enough for the UE-side pre-compensation of AP-42. What it does not buy:
64-QAM or denser (about 2 degrees rms is needed), anything coherent ACROSS
nodes (every system above that beamforms from independent oscillators uses
references every 1 to 40 ms AND a predictor), or immunity to a scatterer or
Doppler change inside the frame.

NR's answer is DMRS plus PTRS inside the slot; 802.11's is four pilot
subcarriers in every symbol; this repo's MATLAB reference ships with the
per-symbol pilot correction as the tier that carries it (section 5). So the
durable design is unchanged from section 9: the beacon supplies time,
frequency and a coarse per-frame phase anchor, and a few pilot tones in each
downlink data symbol supply the common-phase-error correction. Downlink data
symbols do not exist yet (AP-45); when they do, the phase tracker is the piece
to build. The cheap measurement that would size the pilot density BEFORE then
is the beacon-phase innovation against elapsed frames, k = 1 to 8, which
`beacon_phase_coherence.py` already measures at k = 1.

Sources: TS 38.211 (7.4.2.2 PSS, 7.4.2.3 SSS, 7.4.3 PBCH, 5.2.1 Gold
sequence), TS 38.213 4.1, TS 38.214 5.1.6.1.1;
https://www.mathworks.com/help/5g/ug/nr-cell-search-and-mib-and-sib1-recovery.html;
srsRAN 4G `lib/src/phy/sync/ssb.c` and `pss_nr.c`;
https://arxiv.org/abs/1205.6862 (AirSync); https://arxiv.org/abs/2311.11070
(BeamSync); https://arxiv.org/abs/2506.07267 (Merlo et al.);
https://arxiv.org/abs/2509.03722 (OTA phase calibration in TDD).
