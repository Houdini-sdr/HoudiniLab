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
