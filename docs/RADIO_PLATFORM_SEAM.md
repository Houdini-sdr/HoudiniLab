# The radio platform seam: design and verification plan

Status: PLAN, 2026-09-03, decided with the user after the review-fix round
(DEMO_VERIFICATION 8.175 to 8.182). Companion to
`SYNC_LIBRARY_ARCHITECTURE.md` (the sync library) and
`SOUNDER_CHANGE_PACKAGING.md` (fixes against features).

## 1. Decisions taken

1. **Agora's radio layer: take what is rigorous, improve the rest, keep our
   code.** Agora (github.com/Agora-wireless/Agora, `src/agora/radio`) shares
   this sounder's RENEW heritage and is the more refined layer, but it is
   not a golden reference [user 2026-09-03]. Worth taking: an abstract
   `Radio` with a runtime factory over backends (Soapy stream, Soapy socket,
   native UHD), one set class per role (`RadioSet`, `RadioSetBs`,
   `RadioSetUhd`, `RadioSetCalibrate`), and lifecycle names that read as
   what they do (`Init`, `Setup`, `Activate`, `Deactivate`, `Close`,
   `Flush`). Worth doing better: its base class carries three `Rx`
   overloads, an `Init` that takes the whole `Config`, and Iris framer hooks
   (`ConfigureTddModeBs`, `Trigger`, `AdjustDelay`, `HwFramer`) as virtuals
   with no-op defaults, so every backend inherits another platform's
   concepts. Our version keeps `Radio` narrow (streams, time, device facts,
   one receive form), passes a `RadioParams` value instead of the config,
   and puts the framer, the transmit grid and the gap ledger in explicit
   capability objects a backend either provides or does not. Pulling Agora's
   code in now would mean re-validating the Houdini path against a foreign
   implementation with no Iris or UHD hardware to check the other side; the
   same roles and names make a later exchange in either direction mechanical.

2. **Environment overrides of the sync knobs default to off.** The bench
   scripts write their sweeps into the JSON overlay instead
   (`run_shape_campaign.sh`, `SYNC_OVERLAY`). Every override is still logged
   when a config turns them back on.
3. **Tree hygiene at branch landing, not before.** The ledger and the
   walkthrough stay (they are the verification record of this code); the
   raw evidence captures and the superseded bench probes move out or are
   pruned when the branch lands.
4. **Iris and UHD stay inspection-only until hardware exists.** What can be
   done without hardware is a build matrix over the three `RADIO_TYPE`
   values, so a change that breaks a compile is caught on the day it lands
   rather than found by a reviewer months later (baseline B1).
5. **The 20-byte packet and the portable correlator are accepted** for every
   backend; the AVX kernels stay under the bench test.
6. **P3 (the false-alarm-derived coherence bar) follows the seam**, then
   AP-72's measurement.

## 2. What the seam abstracts

Both Iris and Houdini speak SoapySDR; the pure-UHD path speaks libuhd
directly and is selected at compile time, which is why it was the path that
stopped compiling unnoticed. The device-specific features, read out of the 48
`is_houdini()` sites in the tree:

| Concern | Iris | Houdini | Where today |
|---|---|---|---|
| Device open and stream arguments | serial, defaults | driver, remote address, MTS membership stream, replay-mode TX, host ports | `BaseRadioSet.cc` 1070, `ClientRadioSet.cc` 313 |
| Rates before streams open | set in `dev_init` | rate and NCO set before `setupStream` (no live rate change) | `Radio` ctor arguments |
| Gain stages, AGC, sensors | present | none | `BaseRadioSet.cc` 174, `ClientRadioSet.cc` 107, 386 |
| Beacon transmission | software per-frame TX through the Iris TDD framer | device replay RAM, armed once | `BaseRadioSet.cc` 264, 1292 |
| Base-station framer | Iris TDD JSON, triggers, sync delays | native TDD ring, strobe grid, gated RX, frame-tagged windows | `BaseRadioSet.cc` 480-1000, 1220-1350 |
| Client acquisition | hardware trigger and correlator | software beacon search, stamp-anchored | `ClientRadioSet.cc` 267; `receiver.cc` |
| Transmit time grid | whole milliseconds | 3125 ns (384-tick) snap, tick advance | `ClientRadioSet.cc` 463 |
| Receive drain and gap ledger | none | dropped-packet zero-pad, pad count, TX status events | `Radio.cc` recvHoudini, `lastPadSamples`, `drainTxStatus` |
| Sync defaults | first crossing, power ratio | first path, normalised cross-correlation | `SyncConfig::resolve(Platform)` (done) |
| Recorder | none | conjugate the R2C mixer, CSI timing and phase fixes, gaps table | `recorder_worker.cc` 95-115, 1157 |

## 3. Target shape

```
struct RadioParams {                // what a radio needs to open: no Config pointer
  std::string address; std::vector<size_t> channels; double rate, nco, rf_freq;
  double rx_gain, tx_gain; std::string bw / antenna facts; stream arguments
};
class Radio {                       // abstract and NARROW: streams, time, device facts
  enum class Type { kSoapyIris, kSoapyHoudini, kUhdNative };
  static std::unique_ptr<Radio> create(Type, const RadioParams&);   // the one place that knows the type
  // pure virtual: recv (one form), xmit, activateRecv/Xmit, deactivateRecv/Xmit,
  //   getTriggers, setup (gains), drain_buffers
  // device facts: platform() -> houdini::sync::Platform, type(), serial()
  // capabilities a backend provides or reports absent (no silent no-ops):
  //   txGrid()   -> the transmit time grid (Houdini: 3125 ns snap, tick advance)
  //   gapLedger()-> pad count and TX status (Houdini); absent elsewhere
};
class RadioSoapy : Radio            // the Soapy plumbing, Iris behaviour (today's Radio minus the branch)
class RadioHoudini : RadioSoapy     // stream arguments, pre-stream rates, the drain, the grid snap, status
class RadioUhd : Radio              // today's RadioUHD, under USE_UHD

class BeaconFramer {                // what the base station does per frame
  arm(), rx(radio, buffs, frameTime), stop(), framePad()
};
class IrisTddFramer : BeaconFramer  // the TDD JSON, triggers, per-frame beacon TX
class HoudiniTddFramer : BeaconFramer  // the ring, the strobe grid, the gated RX (today's htdd_* block)

BaseRadioSet, ClientRadioSet        // hold std::unique_ptr<Radio>; pick the framer from the radio's platform;
                                    // no is_houdini() left in either
```

The receiver keeps two acquisition models (the stamp-anchored Houdini one
and the trigger-driven Iris one) because they are different algorithms, not
different devices; its remaining branches are keyed on the library's
`Platform`, and unifying them is a later phase.

## 4. Steps and gates

Each step is behaviour-preserving on Houdini at the shipped defaults and
gated the way P1 was (DEMO_VERIFICATION 8aa): the six suites, the 30 golden
windows on x86 and aarch64, and interleaved silicon runs. Iris and UHD get
the build matrix.

| Step | Content | Gate |
|---|---|---|
| S0 | `tools/build_matrix.sh` (SOAPY_IRIS, SOAPY_UHD, PURE_UHD where libuhd exists, else reported as skipped); env overrides off + the campaign overlay | matrix green; suites; the campaign script runs a sweep through the overlay |
| S1 | abstract `Radio`, `RadioSoapy`, `RadioHoudini`, the factory; the sets hold `unique_ptr<Radio>`; the Houdini radio-level branches move into `RadioHoudini` | matrix; suites; 30 windows; 3 interleaved runs (legacy, nr_pss, dot11) |
| S2 | `BeaconFramer` with the Iris and Houdini implementations; the base station's framer block moves out of `BaseRadioSet` | same |
| S3 | `RadioUhd` behind the same sets; the UHD set classes and the receiver's `#if USE_UHD` type switch go | matrix (compile-only for UHD); suites; 3 runs |
| S4 | the receiver's remaining branches keyed on `Platform`; the recorder's Houdini fixes as a `PlatformRxFixes` object | same |

Reviews: Opus after each step until a round reports nothing new, as for
the library. Retractions and corrections go in the ledger as before.

## 5. Out of scope here

Agora's socket data plane, a runtime choice of pure UHD (it stays a compile
option until it can be tested), and the receiver's acquisition-model
unification.
