# The eps cross-check: what to run, and what each outcome means

Written before the run, deliberately, so the interpretation is not chosen after
seeing the numbers.

## The problem

At the calibrated pair's ~0.25 ppm our four clock instruments span 0.115 to
0.318 ppm (DEMO_VERIFICATION 8.89):

| instrument | eps | how it measures |
| --- | --- | --- |
| sounder grid tracker | +0.115 ppm | predicts the beacon, measures the miss |
| sounder acquisition | +0.115 ppm | confirms over ~200 frames, wide window |
| `clock_drift_probe` arrival ramp | +0.254 ppm | least-squares over ~4000 detections in 60 s |
| software lane RF tone | +0.318 ppm | independent carrier mechanism |
| `hwtime_rate_probe` | +0.728 / -0.137 | no resolution here (8.91) |

The two sounder-side numbers agree with each other and disagree with the two
RF-side numbers by about 0.14 ppm.

## Why the obvious tie-breakers do not work

- **The RF-free ratio has no resolution at this eps** (8.91). It differences two
  host-referenced rates that each wander ~3 ppm between runs.
- **The tracker's residual test is not circular**, which was the first
  objection raised. The residual is `x - period * round(x / period)` over `n`
  frames since the last re-anchor, so a period error `d` appears as
  `n * period * d`. At n = 412 and d = 0.139 ppm that is 7.0 samples, and with
  alpha = 0.5 the steady state would be 14.2. Observed: mean +0.08, max +-2.
- **Running more of the same instruments does not help.** They are already
  self-consistent and repeatable; repetition tightens error bars that were
  never the problem.

## The experiment

Run BOTH families at a LARGE eps and again at a SMALL one. Internal clocks give
about 8.5 ppm (AP-33 measured -8.52); calibrated gives about 0.25.

Per clock mode, on ONE instrument build, ideally interleaved:
1. `clock_drift_probe` x3, 60 s each -> arrival-ramp eps
2. `sounder --view` x3, 65 s each -> the tracker's converged eps and its
   acquisition bootstrap period, from the log
3. the software lane's tone, if they will run it, for a third channel

## What each outcome means, decided now

| outcome at 8.5 ppm | reading |
| --- | --- |
| the two families AGREE to ~0.05 ppm | the discrepancy is an **additive offset** of ~0.14 ppm in one channel, invisible at 8.5 ppm (1.6 %) and dominant at 0.25 (55 %). Then measure the offset at both scales and find which channel carries it. |
| the two families disagree by the SAME 2.2x RATIO | a **scale error**, which would also mean the original campaign's "three instruments agree to 0.02 ppm" was wrong, since it was taken at this eps. Re-derive that closure before anything else. |
| they disagree by some OTHER amount | neither model fits; stop and characterise before drawing any conclusion. |

## What is already decided regardless of outcome

**No sub-ppm clock figure from this bench is currently better than +-0.1 ppm**,
including the +0.261 and +0.084 ppm this lane supplied to the software lane's
SH-339/SH-341 acceptance check. Those were sent with their run-to-run spread
attached, and the spread was honest, but the INSTRUMENT disagreement was not
known at the time and is larger than the spread. The software lane has been
told.

## What this does NOT test

The beacon carrier channel's bias (AP-39, confirmed live at +0.49 to +1.21 ppm
above the timing truth) is a separate, known effect and is not what this is
about. This is a disagreement between two TIMING measurements.
