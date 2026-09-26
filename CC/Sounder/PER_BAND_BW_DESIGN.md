# Per-band channel widths (AP-85)

Status: implemented and in the demo head (`channel_ofdm_data_num`, used by
every `houdini-dualband-xw*` config). The rig predictions of section 5 ran as
`DEMO_VERIFICATION.md` 9.55: P1 to P4, P6, P8 and P9 held, P5 held only in
part, P7 and P10 were not evaluated. Sections 1 to 4 are the reference for the
key; sections 5 to 7 are the design's record. Q3's framer placement is
superseded by AP-90 (the note at Q3).

The X-band carries 270 RB (3240 subcarriers, 97.2 MHz occupied) beside the sub-6's
133 RB (1596 subcarriers, 47.88 MHz), on ONE numerology: SCS 30 kHz, FFT 4096 at
122.88 Msps, CP 288, 14 symbols per 0.5 ms slot, the same TDD schedule, timing and
stream rates. Only the X-band's tone count, and what derives from it, changes.

Requirement: HS-202, in the Houdini-Streaming repository:
`docs/DEMO_FREQUENCY_PLAN.md` section 3.3 (the numerology table,
"X-band later, near 100 MHz: 270 RB adopted"), section 6.1 (the W3 note on the
refclk image at +43.68 MHz), section 9b ("X-band channel near 100 MHz [user]"),
section 1 and `docs/XUD1A_FREQUENCY_PLAN.md` (IF 4380, 98.3 MHz usable, 4330.85 to
4429.15 MHz).

Scope [user]: each band carries its own independent pilot and data at its own
width. The milestone line "the same pilot and data on both bands" is not a
constraint. Smallest correct design; no option (b).

## 1. What changes, in one paragraph

A new optional config key `channel_ofdm_data_num` gives a per-channel tone count,
keyed by channel letter exactly like `channel_nco_frequency`. Config builds one
OFDM "band" (pilot, data, their indices and level) per distinct tone count; a
channel without an override uses the default band (`ofdm_data_num`), which is
built by the same code as today, so a config without the key is byte-identical.
The UE's seated burst copies each TX lane's own band into its P and U slots. The
BS recorder computes each antenna's CSI and constellation against its RX
channel's band. The mode-V plan checks each channel against its own occupied
band. The x2 TX interpolator is chosen per TX lane: a lane whose band fits the
+-24 MHz channel filter keeps today's path (channel-filter prefilter, then the
23-tap halfband); a wider lane takes a new wide halfband (47 taps) with no
prefilter. The Zadoff-Chu length rule is fixed above 2039 tones (it silently
capped the prime at the end of a table). A new config
`files/houdini-dualband-xw.json` sets `{"B": 3240, "C": 3240}`.

## 2. Design questions

### Q1. Config shape and backward compatibility

`"channel_ofdm_data_num": {"B": 3240, "C": 3240}`: channel letter to tone count,
the idiom `channel_nco_frequency` already uses [user: keep the config common,
split only what must differ]. Both nodes load the same file, so the letter names
the converter channel on whichever node opens it: TX ch1 (B) on the UE, RX ch2 (C)
on the BS. Absent, every channel uses `ofdm_data_num`.

Rules (each refuses at config load, naming the key):
- mode V only, like `channel_nco_frequency` (only the mode-V bring-up applies a
  per-channel plan);
- one letter A-D per key, a whole number of RBs (a multiple of 12 tones: the
  pilot tones sit at each RB's centre, and the ZC generator reads one element
  past its sequence for an odd count), at most `fft_size`, and a ZC pilot
  (`fft_size` other than 64, whose 802.11 LTS has a fixed 52-tone layout);
- **the tie between UE TX B and BS RX C**: channels on the same NCO carry the
  same tone count. In mode V a channel's NCO is its band's centre, so two
  channels on one NCO are one band; the UE's B and the BS's C are both 4380, and
  a file that sets B but forgets C (or the reverse) is refused, naming both
  letters. This is what stops the BS from estimating the X-band against the
  wrong pilot, a failure that would otherwise surface on the rig as a garbage
  X-band panel;
- in mode V, every channel's occupied half band (default or override) at most
  0.4 x `sample_rate` = 49.152 MHz: the RFDC decimator's 80 % passband (PG269 p.77),
  which is also the wide TX halfband's design edge (Q5). 273 RB (+-49.14) passes,
  anything wider is refused.

Backward compatibility, pinned: `tests/comms-func/per_band_test.cc` hashes every
band-0 buffer Config builds (pilot slot, UE data slot, the pilot's frequency and
time symbols, the pilot tone values and indices, the data indices, the Iris RAM
image, the beacon, `tx_scale`, the occupied half band, the tone count, the
written `ul_data_f_*.bin`) plus the prefiltered interpolator's output on the pilot
slot, for `houdini-dualband.json` and five other shipped configs, and compares
each with the value measured on the untouched baseline (`d27d5f5`), plus an
Iris-scaled variant of `houdini-r0.json` and the ZC generator at 96 to 2048
tones (both sides of its table's end). The demo configs
`files/houdini-dualband*.json` are not modified. Precisely: every SHIPPED
config is byte-identical; a config without the key changes only if its
`ofdm_data_num` exceeds 2039 (the ZC fix) or, in mode V, 3276 (the passband
rule), and no shipped config comes near (the largest is 1596).

### Q2. The pilot on 3240 tones

Construction: unchanged, the ZC tones on the centre `n` subcarriers of the
fft-4096 grid (start (4096 - n) / 2 = 428), DC included exactly as the sub-6
pilot has run it at every rung (no new DC handling). The data symbols' pilot
tones are each RB's centre (grid index 434 + 12 k, 270 of them), valued from
the pilot grid, as today.

**Length rule defect, fixed.** `CommsLib::getSequence(LTE_ZADOFF_CHU*)` takes the
largest prime below the length from a 309-entry table ending at 2039. For any
length above 2040 the lookup falls through to 2039, so 3240 tones would be a
length-2039 ZC cyclically extended by 1201 tones (59 %). Every tone keeps unit
magnitude, so the CSI would still be right, but the time-domain PAPR rises from
3.69 dB (the proper prime, 3229) to 5.26 dB; the pilot is peak-normalized, so
that is 1.57 dB of X-band pilot power lost for nothing. The fix computes the
largest prime below the length by trial division when the length exceeds the
table's last prime; every length the table covers takes the table path,
unchanged. Measured PAPR (fft 4096, DC-centred grid): 1596 tones 3.92 dB, 3240
tones 3.69 dB (5.26 with the defect), 3276 tones 3.00 dB (5.32).

**Level.** The OFDM pilot level is PEAK-normalized: `tx_scale` = 1 / (2 x the
pilot's peak), so the pilot peaks at half full scale, and the U slot is scaled
to the pilot's realized peak [user, AP-79]. Each band is normalized on its own
(a configured nonzero `tx_scale` still applies to every band). (The beacon's
`sync.beacon.tx_full_scale` is a separate, sub-6-only scale, unchanged.) At the
same peak, per-tone amplitude is 1 / (2 sqrt(PAPR x n)): the 3240-tone pilot
carries 10 log10(3240 / 1596) - (3.92 - 3.69) = 2.85 dB less power per tone than
today's 1596-tone X-band pilot, and its RMS (the DAC load) is 0.23 dB higher.
That is the physics of spreading the same DAC power over twice the band; the
X-band's per-tone SNR drops by the same 2.85 dB (the noise per tone is
unchanged). The exact figure from the built waveforms is in the test output and
in the rig predictions (section 5).

### Q3. The BS framer's pilot edge and `tx_advance`

Superseded in part by AP-90: the lane with the cleanest pilot now places the
cut (`slot_align.h` `laneTakesCut`), not lane 0. On the direct X-IF cable that
is usually the X-band lane (`DEMO_VERIFICATION.md` 9.69: 2,536 of 2,545 cuts),
and `pilot_grid_off` stayed within a few samples (9.69 to 9.71), so the
`tx_advance` calibration held. The reasoning below is the design as written.

`HoudiniFramer` finds the burst (energy search, presence gate, P/U
self-similarity, the leading edge in `slot_align.h`) on lane 0 of the combined
RX stream only, and applies that placement to every lane (the lanes are
sample-aligned). In every dual-band config lane 0 is RX ch A, the sub-6, whose
pilot is byte-identical. So the edge, `pilot_grid_off`, and the `tx_advance`
calibration (169) do not move with the X-band's width. The X-band lane's content
is placed by the same offsets; its TX lane changes from (prefilter + 23-tap
halfband) to the 47-tap halfband, both zero phase, so no delay enters (P2b
checks it on the X-band lane itself).

What the rig must re-measure: nothing is recalibrated, but `pilot_grid_off` is
logged (HOUDINI_BS_RX_DEBUG=1) and compared against the 1596 control (prediction
P2). If a config ever put the X-band on lane 0 (`rx_channel` "CA"), the framer
would edge-detect on the wide pilot and 169 would need re-deriving; no shipped
config does, and the design note records it rather than guarding it.

### Q4. Data payload policy (decided [user]: independent per band)

Each band carries its own random QPSK U slot at its own width (2970 data tones
plus 270 pilot tones on the X-band), built by the same code as the sub-6 one
(fixed seed, a distinct random symbol per OFDM symbol, peak-normalized to its
band's pilot). The same seed is used per band; the bands are separated by
channel at the BS, so nothing couples them.

### Q5. The wide interpolator and its host cost

Design: halfband `HalfbandInterp2(47, 7.0)`, Kaiser-windowed, zero phase,
passthrough samples bit-exact (the existing class, other taps). Sized by a sweep
(tones through `runCircular`, the existing measurement) over content to
+-49.152 MHz, whose images start at 122.88 - 49.152 = 73.728 MHz:

| taps | best beta | worst image | ripple |
|---|---|---|---|
| 39 | 6.0 | -60.3 dB | 0.015 dB |
| 43 | 6.5 | -66.1 dB | 0.007 dB |
| **47** | **7.0** | **-70.5 dB** | **0.0044 dB** |
| 51 | 7.5 | -75.6 dB | 0.0026 dB |

47 is the smallest meeting the requirement (60 dB, software lane) plus the
10 dB margin the existing design documents (fine grid over +-49.152 MHz:
-70.51 dB; the narrow 23-tap design over the same band: -17 dB). Its zero-margin context is 12 input
samples before and 11 after (the narrow prefiltered path needs 23 / 22), inside
the slot's 32-sample zero prefix and postfix; the config rule now takes the
larger of the two.

Host cost: the UE re-sends the same content every frame, and the interpolator
already computes the content once and places it (a memcmp and a memcpy) on every
later burst, whatever the filter. So the steady-state TX cost is unchanged; only
the first burst per lane (and any content change) runs the filter. Microbenchmark
(`tx_interp_bench`, this VM, x86, one lane, the demo's P+U burst of 122980
input samples, 21 interleaved rounds, medians):

| path | time |
|---|---|
| today's X-band lane: prefilter + 23-tap halfband | 4.64 ms |
| 23-tap halfband alone | 3.25 ms |
| wide lane: 47-tap halfband alone | 4.02 ms |
| steady state, placement hit | 0.24 ms |

So the wide lane's first burst costs 0.87 x today's X-band lane (it drops the
prefilter) and 1.24 x the 23-tap halfband alone.
`tx_interp_bench` (built by default, not in ctest) prints the same table on the
rig host: `build/tx_interp_bench`.

No prefilter on the wide lane: the prefilter exists because sub-6 splatter 40.2
to 61.4 MHz from the NCO folds back ON the channel through the far ADC's
real-sampling mirror. The X-band's mirror sits more than 1 GHz away (rf_plan: no
RX filter on its lane), and a +-49 MHz band leaves no room for a channel filter
below Nyquist anyway. Splatter between 48.6 and 73.7 MHz meets the halfband's
transition band and the far decimator; none of it lands on an occupied tone.

### Q6. What else the tone count reaches

See section 3, the call-site table.

## 3. Call sites

Found by a survey of every file type in the repo (C++, Python, shell, JSON,
markdown), then verified here. "Changes" are in this branch; "adapts" means it
follows the per-antenna data with no change; "not on the path" means Iris-only,
one-rate only, or the sub-6 beacon.

| Where | What assumes one width | Disposition |
|---|---|---|
| `config.cc` / `config.h`: `genPilots` | one pilot, one data slot, one `data_ind` / `pilot_sc*`, one `tx_scale` | Changes: `OfdmBand` per distinct tone count, built by one function; the legacy accessors are band 0 |
| `config.cc` mode-V rules (216-230) | the zero prefix/postfix covers the prefiltered interpolator's margins only | Changes: the larger of the two paths' margins |
| `comms-lib.cc` `getSequence` ZC | the prime table ends at 2039 | Changes: Q2 |
| `receiver.cc` `initBuffers`, seated burst (1095-1120) | one `pilot_ci16`, ch A's data on every lane | Changes: each TX lane copies its channel's band; the legacy non-horizon path's per-lane buffers likewise |
| `recorder_worker.cc` `initCsi`, `sendCsi`, `sendMeta`, `sendConstellation` | one `pilot_ref_`, one `data_ind`, one `pilot_sc*` | Changes: per BS antenna, from its RX channel's band |
| `recorder_worker.cc` HDF5 attributes | one `OFDM_*` set per file | Changes: recording mode is refused when the bands differ (Q6 note below) |
| `BaseRadioSet.cc`, `ClientRadioSet.cc`, `Radio.h`, `mode_v_bringup.h`, `RadioHoudini::modeVPlan` | one `half_bw_hz` for every channel's plan | Changes: `half_bw_by_channel`, beside `nco_by_channel` |
| `RadioHoudini.cc` (302-304), `tx_rx_boundary.h` `TxBurstInterpolator` | one prefilter flag, one halfband for every TX lane of a radio | Changes: one interpolator per TX lane, chosen by `TxBurstInterpolator::forBand(half_bw)` |
| `dsp/band_filters.h` | the only halfband is designed for +-25 MHz | Changes: `HalfbandInterp2::wide()`, 47 taps |
| `HoudiniFramer.cc` energy search, presence gate, self-similarity, leading edge, `beaconLeadTicks` | all on lane 0 (sub-6) | Adapts (Q3); the beacon lead keeps the prefiltered figure |
| `cir.h` Hann window | the non-zero span of H | Adapts per antenna |
| `csi_gui/csi_server.py` | each card's axis from its own CSI2 frame (4096 bins, unused tones drawn as gaps); MET1 per antenna feeds the RB/bandwidth line; one global `dashboard_mag_top` | Adapts; the X-band trace spans twice the bins; the 3 dB lower X-band level stays inside the 75-115 dB window |
| `tests/demo-verify/rig_dumps.py`, `fstage_report.py` `ul()`, `fstage_hratio.py`, `ab_report.py`, `mer_sampler.py` | read N and `data_ind` from each dump | Adapts (the dumps carry the antenna's own `data_ind`) |
| `tests/demo-verify/fstage_report.py` `stage_stats` | refuses a rung tag reused across numerologies by `N` only (4096 either way) | Not changed: give the 270 RB runs their own tag (the rig plan does); recorded as a residual |
| `evm_compare.py`, `ap15_diff.py` | take the first `ul_data_f_*.bin` as THE reference | Not on the path (antenna-0 bsframe tools); no X-band reference file is written, so the first stays the sub-6's |
| `PYTHON/IrisUtils/plot_hdf5.py`, `hdf5_lib.py`, `csi_analysis.py` | one `OFDM_PILOT_F` / `OFDM_DATA_SC` for all antennas | Not on the path: recording mode is refused for differing bands, and these tools already fail at fft 4096 with 1596 tones (`hdf5_lib.py` sizes the FFT as the power of two above the tone count) |
| `config_pilot_test.cc` | the pilot checked inside +-24 MHz | Still right for band 0; the new config is added to its list; the X-band band is tested in `per_band_test` |
| `mode_v_bringup_test.cc`, `rf_plan_test.cc`, `band_filters_test.cc`, `tx_rx_boundary_test.cc` | one width, +-25 MHz | Extended (section 4) |
| `csi_gui/fake_feed.py`, `test_metrics.py` MET1 fixtures | 1596 on every card | Not changed: they mirror `houdini-dualband.json`, which is unchanged |
| `data_generator.cc`, `hdf5_reader.cc`, Iris framer and calibration, the 802.11 fft-64 tools, `tests/hil` | Iris / one-rate / fft 64 | Not on the path; the new key is refused outside mode V |
| `files/houdini-dualband.json` description "133 RB", `CSI_DEMO_WALKTHROUGH.md` "133 RB" | describe that file | Still true of that file; the new config carries its own description |

**Recording mode (HDF5).** The file's `OFDM_PILOT_F`, `OFDM_DATA_SC`,
`OFDM_PILOT_SC(_VALS)` and `DATA_SUBCARRIER_NUM` describe one band. With two
bands a file would describe the X-band antenna with the sub-6 pilot, and the
offline tools would compute a wrong channel without saying so. So the recorder
refuses recording mode when the bands differ and names view mode and the CSI
dumps as the path; the demo and every AP-79 run use view mode. A user decision if
raw recording at 270 RB is wanted (section 6).

## 4. Test plan

Every check is known-answer, and names the mutation that breaks it; each
mutation is run once and must fail its check, then is restored.

`tests/comms-func/per_band_test.cc` (new, links the sounder's sources, runs from
`CC/Sounder`):
1. **Backward compatibility pin.** For `houdini-dualband.json`, `-40`, `-r3a`,
   `-r2`, `-r1` and `houdini-r0.json`: one FNV-1a hash over every band-0 buffer
   (the list in Q1) and one over the prefiltered interpolator's output on the
   pilot slot, equal to the values measured on `d27d5f5`. Mutations: the default
   band built with another data seed; the default band built with an override's
   tone count.
2. **The new config's mapping.** `houdini-dualband-xw.json`: channel A's band is
   1596 tones and hashes equal to `houdini-dualband.json`'s band 0 (an override
   does not touch the default band); B and C share one 3240-tone band; the BS
   antenna map (antenna 1 is RX ch C) and the UE TX lane map (lane 1 is TX ch B)
   give 3240. A probe config (C overridden at its own NCO, B left at the default
   NCO and width) separates the letter from the lane index: BS antenna 1 must be
   3240 and UE TX lane 1 must be 1596. Mutation: mapping by lane index instead of
   channel letter.
3. **The wide pilot.** 3240 unit-magnitude tones on grid indices 428..3667, zero
   elsewhere; the pilot slot one slot long; energy outside +-48.6 MHz under
   -40 dB; the ZC period 3229 (tone 3229 equals tone 0); PAPR at most 4.0 dB.
   Mutation: the table-capped prime (period 2039, PAPR 5.26 dB).
4. **The wide data slot.** 2970 data tones (428 to 3667) and 270 pilot tones
   (434 + 12 k); the built U slot transmits every pilot tone; its realized peak
   equals the band's pilot peak. Mutation: the data slot not normalized to its
   pilot's peak.
5. **Levels.** Each band's pilot peaks at half full scale (16384 counts, within
   rounding); the per-tone amplitude ratio X-band over sub-6 printed and checked
   inside -3.0 to -2.7 dB. Mutation: band C built with band 0's `tx_scale`.
6. **Refusals**, each on a temporary config derived from the new one: the key
   without mode V; B overridden without C (same NCO, different counts; the
   message names both); 3300 tones (+-49.5 MHz over 49.152) refused while 3276
   (273 RB) loads; a count of 0, above `fft_size`, or not an integer; a key that
   is not one letter A-D; `fft_size` 64 with the key. Mutation per refusal: that
   check removed.
7. **The radio parameters.** `channel_half_bw()` holds B and C at 48.6 MHz
   exactly and nothing for A. Mutation: the map filled from band 0.
8. **The BS recorder end to end** (view mode, UDP loopback, the real
   `RecorderWorker`): antenna 0 fed the sub-6 pilot slot and antenna 1 the
   X-band one, each conjugated as the RX path delivers it. CSI2 |H| is flat
   (1 %) on exactly 1596 and 3240 tones and 0 elsewhere, and the X-band's
   level sits at the per-tone amplitude ratio (-2.85 dB, P5's instrument);
   MET1 reports 1596 and 3240 tones, 47.88 and 97.2 MHz; after the U slots,
   each antenna's CNS1 constellation is QPSK at MER 40 dB or better. Mutation:
   the recorder using band 0 for every antenna.
The ZC generator below its table is covered by item 1 (96, 1272 and 1596 tones).

`band_filters_test.cc`: the wide halfband measured the existing way over
+-49.152 MHz: images at least 60 dB down and at least 70 (the margin; 70.5 measured), ripple at most
0.01 dB, zero phase, passthrough bit-exact, margins tight and inside the 32-tick
prefix/postfix. Mutation: the 23-tap halfband measured over the wide band
(images 39 dB).

`tx_rx_boundary_test.cc`: `forBand(23.94 MHz)` is bit-identical to today's
prefiltered interpolator on a burst (the demo's path unchanged); `forBand(48.6
MHz)` passes a 45 MHz tone within 0.01 dB (mutation: the prefiltered path chosen
for the wide band cuts it by more than 40 dB); placement equals the full
computation for the wide lane at every pad 0..383; `maxLead()` / `maxTail()`
cover both paths.

`mode_v_bringup_test.cc` / `rf_plan_test.cc`: the UE plan with TX ch1 at
+-48.6 MHz (zone 2) and the BS plan with RX ch2 at +-48.6 MHz (zone 2, cal mode
2, no filter) pass beside ch0 at +-23.94; the same width on ch0 is refused
(over the channel filter's passband). Mutation: one width for every channel.

`config_pilot_test.cc`: the new config added (its band 0).

`tx_interp_bench` (standalone): the two lane paths and the placement hit on the
demo's P+U burst, interleaved over rounds, medians and ratios printed.

## 5. Rig alpha-test plan, predictions registered before the run

Setup: the AP-79 V1/V2 stack, layout and wiring (`.22` BS, `.21` UE, the rig
host `.26`), preflight as `DEMO_BENCH_RUNBOOK.md`. Interleaved A/B, the same
session: B = `files/houdini-dualband.json` (the control), X =
`files/houdini-dualband-xw.json`, in the order X, B, X, B, X, B, each about
5 minutes with `HOUDINI_BS_RX_DEBUG=1` and `HOUDINI_CSI_DUMP` set, the X runs
under their own rung tag (for example `xw`, so `fstage_report.py` never averages
them with 1596-tone runs). Carrier offset is the dominant MER term (9.13, 9.49),
so every MER comparison is at a matched offset window, and one run is not a
behaviour.

First, on the rig host: `build/tx_interp_bench` (no radio), for Q5's numbers on
aarch64.

| # | Prediction | Falsifier |
|---|---|---|
| P1 | X starts; the mode-V plan lines read "UE TX ch1: NCO 4380.000 MHz, occupied +-48.600 MHz, zone 2" and "BS RX ch2: NCO 4380.000 MHz, occupied +-48.600 MHz, zone 2, cal Mode 2, no channel filter", the sub-6 lines "occupied +-23.940 MHz" with the filter on RX ch0; the UE logs "TX ch1: occupied +-48.600 MHz, wide 47-tap halfband" and "Channel B band: 3240 tones" | a refusal, a filter on ch2, or ch0's plan changed |
| P2 | `pilot_grid_off` in X equals B's within 4 samples (mean) and B's spread: the framer places the burst on lane 0, the byte-identical sub-6 | a mean shift over 4 samples |
| P2b | The X-band lane's own timing is unchanged (both TX paths are zero phase): antenna 1's "delay removed" figure on the dashboard phase panel, and its CIR peak tap, equal in X and B within their spread | a shift beyond the spread: a delay entered with the wide halfband |
| P3 | Sub-6 (antenna 0): |H| median, MER, CNS-low rate and the pilot seat are the same in X and B within their run-to-run spread | a difference outside the spread |
| P4 | X-band (antenna 1): the dashboard reads "270 RB x 12 x 30 kHz, 97.20 MHz"; |H| spans DC-centred indices 428..3667 | anything else |
| P5 | X-band |H| median is 2.85 dB lower in X than in B (the per-tone amplitude ratio from the built waveforms; exact figure in `per_band_test`'s output), within 0.5 dB | off by more than 0.5 dB: a level or scale defect |
| P6 | X-band MER at a matched carrier offset: within B's spread, and at most 2.14 dB lower (the U slot's per-data-tone level drop; offset-limited, 9.13: the ICI does not depend on the tone count, so the drop shows only if the link is SNR-limited) | more than 2.14 dB lower, or the MER no longer tracking the offset |
| P7 | The refclk image (+43.68 MHz = exactly 1456 subcarriers from the NCO, -42.3 dBc of a -12 dBFS tone, so about -54.3 dBFS) lands on ONE X-band data tone, DC-centred index 592 (2048 - 1456: zone 2 inverts the sense, W1/W3). It is coherent with the BS clock and rotates 135 degrees a symbol, so the CSI's 12-symbol average suppresses it about 21 dB: NO visible |H| outlier. In the U slot it sits about 3 dB under a data tone (-51.3 dBFS per tone): the per-tone EVM at 592, from `cns_dump_ant1.bin` (decision-directed error per tone over the dumped symbols), stands out by more than 10 dB over the median tone; no other tone does | an EVM outlier elsewhere, or at 3504 (the sense model is wrong); no outlier at 592 means the line enters on the cable and F1's filter took it (record which) |
| P8 | The X-band CIR's direct-path peak narrows about 2x (Hann mainlobe about 2 / 97.2 MHz = 21 ns against 42 ns): the number of taps within 6 dB of the peak, about 2.5 against 5 at 8.14 ns a tap; the dashboard's "resolution" line reads 21 ns | no narrowing |
| P9 | UE host TX timing: late releases, late frames and under-runs per 5 minutes in X within B's range (the steady-state TX cost is the placement, filter-independent); `tx_sat` stays 0 | X outside B's range |
| P10 | Antenna 1's U-slot RMS (`rms` in `fstage_report.py`, from `cns_dump_ant1.bin`) +0.93 dB in X against B (each U slot is peak-normalized on its own; `per_band_test` prints the built ratio, and +0.23 dB for the pilot slot) | off by more than 0.5 dB |

If P2 fails, stop: the placement is not what Q3 says, and 169 is suspect. If P5
fails, compare `tx_scale` and the pilot peak lines in the UE log with the test's
printed values before anything else.

## 6. Open questions and residual risks

1. [user] Recording mode (HDF5) is refused when the bands differ (section 3):
   view mode and the CSI dumps are the path. Say if raw recording at 270 RB is
   wanted; it needs per-antenna OFDM attributes and the offline tools taught to
   read them (they already fail at fft 4096).
2. [user] The refclk image at +43.68 MHz lands on one X-band data tone (index
   592, P7) at about 3 dB below that tone. The plan allows leaving it empty or
   accepting it; this design accepts it (no code). Cost: it caps the X-band MER
   near 37.7 dB if the line is made inside `.22` (F1 decides where it
   originates); negligible at today's offset-limited 17 to 28 dB, material once
   the steering lifts MER. Nulling that one tone is a small follow-up.
3. Q5's host cost on aarch64 is measured by `tx_interp_bench` on the rig; the VM
   says the wide lane's one-off cost is below today's X-band lane's.
4. `rx_channel` with the X-band on lane 0 ("CA") would move the framer's edge to
   the wide pilot and void the 169 calibration (Q3). No config does; not guarded.
   Since AP-90 the cleanest lane places the edge whatever the order (Q3's note).
5. `fstage_report.py` keys its numerology guard on N only; the rig plan gives
   the 270 RB runs their own tag.
6. The outer RBs sit on the decimator's roll-off (plan 3.3). Not predicted in
   level; the CSI dump shows the edge tones' |H|, and P5 uses the median.

## 7. Review log

Process [user, mid-task]: one light design review and one light code review,
must-fix only, no re-review loops.

**Design review (one round, independent reviewer): 3 must-fix, all folded.**
1. The data symbols' pilot tones sit at each RB's centre (434 + 12 k), not from
   428: the design text and test 4's known answer corrected.
2. P10 was the pilot's +0.23 dB read through a panel that shows no RMS: the U
   slot is peak-normalized on its own and moves +0.93 dB; re-registered on the
   CSI dump's `rms`, the figure printed by `per_band_test`.
3. P7 looked for the refclk line in |H|, where the 12-symbol average suppresses
   it 21 dB, and accepted either outcome: re-derived (index 592, a per-tone EVM
   outlier in the U slot, 3504 as the falsifier) and its MER cap recorded.
Should-fix folded: whole-RB refusal (an odd count read past the ZC sequence);
the wide halfband's 10 dB margin asserted (with a 43-tap mutant); the peak
metric named (|z|); an Iris-scaled variant and ZC 2039/2040/2048 added to the
pin; the recording-mode refusal tested (it runs in the RecorderThread
constructor on the main thread); P2b and P8's method added; P6 on the data's
-2.14 dB. Not taken: a pure helper for RadioHoudini's per-lane choice (the
loop indexes the per-channel map by the channel, and `forBand` is tested);
the cross-node config guard (one sounder process drives both nodes from one
file, so the nodes cannot disagree).

**Code review (one round, independent reviewer): 1 must-fix, folded.** P1
cited a planned width the mode-V log never printed (the plan lines carried
NCO, zone, calibration mode and filter only): each TX and RX plan line now
carries its channel's occupied half width, and P1 names the exact lines. The
reviewer traced every named mutation and found none vacuous, and found the
single-band behaviour (one interpolator per lane, the beacon's null lanes,
output-buffer lifetime, the ZC rule to 2053 tones) unchanged.
