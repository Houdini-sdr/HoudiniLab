# Rig session 2026-09-02: the merge gate and eight decision legs

Stack under test, verified from `getHardwareInfo` on both nodes before any run
and matching what the software lane stated at handover:

| | |
| --- | --- |
| device build, .21 and .22 | `98b17f72` |
| host plugin on .64 | `98b17f72`, SoapyHoudiniSDR `feat/sh341-clock-ref-calibrated` |
| fpga | 1.30, `c88e0b5f` |
| clock | `ref=calibrated`, holdover=1, cal_dac 408 on .21 and 404 on .22 |
| application build | HoudiniLab `feat/internal-clock-cfo` @ `10f2e36` |

**THE HOST PLUGIN IS AN UNGATED BRANCH BUILD**, not the `c20d7975` that
DEMO_VERIFICATION 8.51 and 8.52 were taken on. That was a deliberate call
[user, 2026-09-02]: gate on this stack and record it explicitly. Every row this
session produces binds a stack that is not on `develop`, and a later regression
could belong to either change. It is why the PRE/POST control below exists.

## Files

| file | what |
| --- | --- |
| `pre_final.json` | 3 runs on `fd924dd`, the pre-session code, same stack |
| `ab_final.json` | 9 runs on `10f2e36`, alpha-beta (the shipped tracker) |
| `kf_final.json` | 6 runs on `10f2e36`, `HOUDINI_TRACKER=kf` |
| `kf_nogate_final.json` | 1 run, kalman with `HOUDINI_KF_INNOV_GATE=0` |
| `r1_{a,b,c}.json` | software lane reading 1, both nodes on saved codes |
| `r2_{a,b,c,d,e}.json` | software lane reading 2, .22 steered to man_dac 406 |

Produced by `tests/demo-verify/gate_summary.py` and
`tests/demo-verify/clock_drift_probe.py`. The summaries are re-derivable from
the raw logs, which stayed on the rig host at `/tmp/aplogs/` and are NOT
retained; the json here is the durable artifact.

## Two analysis errors caught before anything was cited

Recorded because both were in this session's own instrumentation and both would
have produced a wrong decision.

1. **The CNS low fraction was not a measurement.** The sounder emits two CNS
   log forms: a periodic summary carrying `(D datagrams, L low)`, and a
   per-event warning throttled on a DOUBLING schedule carrying
   `(low occurrence K of D)`. `gate_summary.py` took the max across both, so it
   reported K, which is only ever 1, 2, 4, 8, 16, 32. Every entry in the first
   table was a power of two across 15 runs, with no exceptions. It was about to
   be filed as a CNS regression against 8.51. Fixed to read the summary only.
2. **A sign asserted from a derived quantity, not checked at its source.** I
   told the software lane their "higher code = .22 runs faster" was inverted,
   having inferred the physical direction from the direction our eps moved
   without checking what our eps was defined as. It is written in
   `clock_drift_probe.py`'s own docstring, line 4:
   `epsilon = (f_BS - f_UE)/f_UE` -- identical to theirs, positive meaning the
   DUT is slow. Their statement was correct; ours agreed with it all along.
   The probe's `--self-test` confirms the relation end to end (injected 0,
   +-2.5, +25, +-800 ppm all recover with matching sign). Withdrawn, and the
   consequence is that AP-47's -0.1251 ppm/count is in EPS units and is the
   same actuator and sign as the software lane's +0.129 in DUT-frequency
   units, not a disagreement.
3. **A/B arms were not interleaved.** Bench conditions demonstrably drift here,
   so PRE/POST and alpha-beta/kalman were both re-run interleaved. The
   conclusions held, but they were not entitled to until then.
