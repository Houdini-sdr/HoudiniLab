# Demo verification ledger: houdini-ul.json, config to device to RF

This is the evidence ledger for the full walkthrough of the two node Sounder
demo (BS 168.6.244.21, UE 168.6.244.22, host rig 168.6.244.64). Every claim
carried by code comments, docs, or folklore is treated as unverified until a
row here says otherwise. Statuses:

- VERIFIED-CODE: traced in source, file:line cited, on the branches named below.
- VERIFIED-TEST: an existing committed test covers it and was confirmed current.
- VERIFIED-HW: measured on the live rig this campaign, evidence linked.
- UNVERIFIED: stated by comments/docs only. Do not build on it.
- WAIVED(reason): deliberately not verified; the reason is recorded.

Discipline [user, 2026-08-30]: claims about PURPOSE or INTENT (what something
is "for") are folklore unless behavior-tested. Rows state observable
behaviors. Cited code locations from exploration reports are spot-checked
before a row is written.

Stack under test: fpga v1.30 `c88e0b5f`, device+host 0.2.2 build `c20d7975`,
proto 1.0. HoudiniLab branch `feat/csi-gui-tabler` @ `1ae17ad`; SoapyHoudiniSDR
`develop` @ `d2861dc` (read only reference); Houdini-Streaming (read only
reference). Config: `files/houdini-ul.json` + `files/topology-houdini.json`.

Campaign plan: `~/.claude/plans/elegant-tickling-galaxy.md` (session-local).
Validation scripts: `tests/demo-verify/`.

Target model [user, 2026-08-30]: UE and BS run exactly the same timing and
schedule with TX and RX reversed. B = BS TX / UE RX. P and U = UE TX / BS RX.
The audit in section 3 measures the current implementation against this.

---

## 1. Config layer: houdini-ul.json -> Config

All rows VERIFIED-CODE this session unless marked. References are
`config.cc` on `feat/csi-gui-tabler` @ `1ae17ad`.

| # | Claim | Evidence | Status |
|---|---|---|---|
| 1.1 | Frame = 30 slots, B@0 P@16 U@18 (BS), UE same minus B | frame_schedule parse config.cc:143-175; loadSlots 176-178 | VERIFIED-CODE |
| 1.2 | samps_per_slot = 4096 = 48 sym x (64 fft + 16 cp) + 128 + 128 | config.cc:286-288 | VERIFIED-CODE |
| 1.3 | Frame duration exactly 1 ms (122880 samps @ 122.88 MSPS) | 30 x 4096 = 122880; rate config.cc:88 | VERIFIED-CODE |
| 1.4 | Beacon = 15 x STS(16) + 2 x gold(128) = 496 samps | genPilots config.cc:789-803 | VERIFIED-CODE |
| 1.5 | Behavior: nothing on the Houdini path consumes the 240-sample STS block. The detector correlates only against the gold sequence passed to it (receiver.cc:1334 passes gold_cf32) | find_beacon_avx correlates match_samples=gold only (comms-lib-portable.cc:270-300, spot-checked); AGC init skipped (ClientRadioSet.cc:359-362); dev_init early-returns (Radio.cc:36-43). The "STS is for AGC" purpose statement is comment folklore and is NOT relied on | VERIFIED-CODE |
| 1.6 | Pilot slot = 48 identical LTS symbols, prefix 128, postfix 128, total 4096; padded to FPGA TX_RAM 4096 | genPilots config.cc:848-890 | VERIFIED-CODE |
| 1.7 | UE data slot = 48 DISTINCT random QPSK OFDM symbols, fixed seed 0xC0FFEE, freq-domain reference written for offline demod | config.cc:909-935, loadULData 960-980 | VERIFIED-CODE |
| 1.8 | BS tx/rx gains absent from json, default 20/20 | config.cc:101-118 | VERIFIED-CODE |
| 1.9 | ALL gain fields (BS and UE) are inert end to end: dev_init returns early for houdini AND the driver setGain is a no-op never forwarded to the remote | Radio.cc:36-43; SH host has no setGain override (host/SoapyHoudiniSDR.hpp:105-110), device listGains = {} (device/SoapyHoudiniSDR_settings.cpp:953-958) | VERIFIED-CODE |
| 1.10 | corr_scale_init=10 used for acquisition only; resync uses corr_scale=100 + retry relaxation | receiver.cc:1334-1337 vs 1152-1156 | VERIFIED-CODE |
| 1.11 | tx_advance=135, ue_tx_advance_ticks=0, ue_pilot_horizon=96, max_frame=600 (HOUDINI_MAX_FRAME override) | config.cc:129-134, 199-211 | VERIFIED-CODE |
| 1.12 | freq==nco==500 MHz makes radio_rf_freq_=0; for houdini neither freq_ nor radio_rf_freq_ reaches the device (only nco via setFrequency) | config.cc:284-285; Radio.cc:178-183 passes preStreamFreq=nco | VERIFIED-CODE |
| 1.13 | tx_scale unset -> auto = 1/(4 x max pilot amplitude), 6 dB backoff | config.cc:867-874 | VERIFIED-CODE |

Notes:
- 1.5 also implies the beacon's only demo function today is the UE frame
  timing anchor. Design audit in section 4.
- 1.8/1.9: signal level is set solely by waveform scale + cabling.

## 2. BS device init, one API call at a time

Call order as made by the code (Radio.cc:139-198): Device::make ->
setSampleRate(RX,ch1) -> setSampleRate(TX,ch1) -> setFrequency(RX,ch1,500e6)
-> setFrequency(TX,ch1,500e6) -> setupStream(RX) -> setupStream(TX replay).
dev_init makes NO further device calls for houdini (Radio.cc:36-43).

Instrument: `tests/demo-verify/bs_init_walk.py`, replaying exactly that
sequence with per-call state snapshots. Evidence: two identical consecutive
runs 2026-08-30, `tests/demo-verify/evidence/rfdc-state-bs-20260830.md`
(full values) + jsonl on the rig.

| # | Claim | Evidence | Status |
|---|---|---|---|
| 2.1 | bs_channel "B" maps to physical channel 1 | utils.cc:29-41 (spot-checked); setupStream on ch1 delivers on FPGA port 10002 (2.7) | VERIFIED-CODE + HW |
| 2.2 | make() restores board default converter state every session: RX 245.76 MSPS, TX 983.04 MSPS, NCO 0. Prior session values do NOT persist | both runs started from identical defaults after a session that left 122.88 MSPS set | VERIFIED-HW (x2) |
| 2.3 | setSampleRate(RX,1,122.88e6) accepted; readback moves immediately; RX_FAB_CLK stays 30.72 MHz | run deltas | VERIFIED-HW (x2) |
| 2.4 | setSampleRate(TX,1,122.88e6) accepted; readback moves; TX_FAB_CLK stays 122.88 MHz | run deltas | VERIFIED-HW (x2) |
| 2.5 | BOTH rate writes are real changes on every sounder start (because of 2.2). The UE construction therefore performs a TX rate change immediately before opening its live TX stream, the documented SH-335 trigger shape | 2.2 + Radio.cc:160-187 order | VERIFIED-HW for the rate-change fact; whether stream corruption follows is section 7 work |
| 2.6 | setFrequency applies the 500 MHz NCO live, before any stream exists; readback ~500e6 (float rounding only) | run deltas | VERIFIED-HW (x2) |
| 2.7 | setupStream(RX, local_port=10002) makes the FPGA egress source port 10002 = 10001 + chan. local_port is channel-coupled: an override that does not match 10001+chan means silent zero delivery | HOUDINI_FPGA_TX_PORT "" -> "10002" observed; coupling claim from driver code (host streaming.cpp:898-906, device fpga/platform.h:762) | VERIFIED-HW (port value); coupling = VERIFIED-CODE |
| 2.8 | setupStream(TX, tx_mode=replay) changes nothing in the readable settings surface | run deltas | VERIFIED-HW (x2) |
| 2.9 | Both nodes run device build 71bcbc6b (2026-08-28T15:45Z), identical stacks, no cross-node skew. But 71bcbc6b is NOT the gate-recorded device build c20d7975 (built 9.6 h earlier); the v1.30 HIL gate does not automatically cover the running build | getHardwareInfo .21 and .22 (2026-08-30) vs SoapyHoudiniSDR TEST_RESULTS.md arc/xband-sw row | VERIFIED-HW both nodes; FLAG open: confirm with the software lane whether 71bcbc6b is a sanctioned post-gate deploy |
| 2.10 | Reading an unknown/write-only setting warns and returns empty; no exception | TX_REPLAY_RANGE read, both runs | VERIFIED-HW |
| 2.11 | Teardown hygiene: closeStream x2 + no arm leaves state=idle, gates_held=0. Side effect: next session's make() reports a latched DAC0.0 FIFO_OVR/UF interrupt from this session's close, on a DAC the session never used | run 2 make() warning | VERIFIED-HW (x1, single observation; re-check) |

| 2.12 | The demo does NOT use MTS on either node. MTS is engaged only by an mts=true setupStream kwarg (fail-loud parser); the sounder passes none | device streaming.cpp:436-462, :947 (spot-checked); BaseRadioSet.cc:791-801, ClientRadioSet.cc:306-312; live: HOUDINI_RX_TARGET_LATENCY=0 after full init, a key valid only under ADC MTS | VERIFIED-CODE + HW |
| 2.13 | The TDD HIL gate rows (D0-D6) also ran WITHOUT mts on their streams, so that evidence matches our configuration. The MTS latency-repeatability result comes from the separate MTS suite and does NOT cover the demo | test_tdd.py stream args (no mts); rx_stream_args = local_port only (houdini_setup.py:484-496) | VERIFIED-CODE |
| 2.14 | Without MTS the ADC pipeline latency is uncalibrated (TARGET_LATENCY invalid) and repeatability across bring-ups is NOT established for our config; make() restarts tiles every session (2.2), so each demo restart is a fresh bring-up. Per-restart latency variation would be a per-run-decided timing constant (AP-15 decision shape) | setting registry :259-266; 2.2 | UNVERIFIED variability; measurement planned (phase 4 timing instrument across restarts) |
| 2.15 | Adopting MTS requires: TX setupStream BEFORE RX (first-up rule, RX mts=true throws if DAC tile 0 unpowered; BS TX-replay setup acquires DAC tiles 0+2), a check whether the UE tx_mode=stream setup joins the group, and teardown care (closing any synced member voids the group) | device streaming.cpp:469-520, :930-947; houdini_setup.py:499-501 | VERIFIED-CODE (constraints); adoption = proposal, not done |
| 2.16 | The deployed v1.30 bitstream on .21 IS MTS-capable, and WITH MTS the converter latency is pinned and repeatable: 6 sync cycles gave identical (lat, off) = adc2 (320, 0), dac0 (512, 0), dac2 (512, 0). REPEATABILITY: PASS | mts_check.py on .21, 2026-08-30 (test 1 + --resets 5) | VERIFIED-HW |
| 2.17 | ADC0.0 and ADC0.1 latch OVR_VOLTAGE + CMODE_OVR + CMODE_UNDR interrupts at every tile bring-up (observed at each mts_check cycle and at walk run 2 construction). ADC0.1 is the demo RX channel. Characterize before drawing conclusions; possibly a benign power-up transient | mts_check.py runs; bs_init_walk run 2 | VERIFIED-HW (occurrence); cause UNVERIFIED |
| 2.18 | OPEN: whether the no-MTS bring-up latency (the demo's configuration) varies across restarts. If it varies, each restart carries a different time-to-sample offset (AP-15 decision shape); if fixed, MTS is not an AP-15 candidate. Measure with the phase 4 sample-exact timing instrument across N restarts | pending | UNVERIFIED, experiment planned |

| 2.19 | RX sample bottom 2 bits are fully alive (uniform 25% per value&3, contiguous values, noise rms 4.15 counts): the decimation DSP fills all 16 bits, so there is NO dead-bit /4 rail in host RX data. Says nothing about the physical converter width; absolute FS mapping (which count = ADC full scale) still unmeasured | rx_bits_probe.py on .21, 253760 values, 2026-08-30 | VERIFIED-HW |

Residual UNVERIFIED for this section: meaning of the "realized/commanded"
wording in the FAB_CLK strings; QMC/mixer/interp state beyond the rate and
NCO readbacks (not exposed by the settings surface; on-board register peek is
the optional independent leg).

## 3. TDD model audit against the target model

(three-column table Iris | current | target; filled in phase 3)

Arming contract, measured live on .21 with `tests/demo-verify/tdd_arm_experiment.py`
(runs 1-3, 2026-08-30; jsonl on the rig under ~/demo-verify-evidence/phase2/):

| # | Claim | Evidence | Status |
|---|---|---|---|
| 3.1 | abort on an IDLE framer does NOT latch gates_held; the sounder's verbatim cold-start sequence (abort -> sched -> strobe -> arm) is accepted | experiment A1, 3 consecutive runs, accepted=1 each | VERIFIED-HW (x3) |
| 3.2 | abort on a RUNNING framer latches gates_held=1, and a re-arm without gate_release is REFUSED and THROWS ("pulse TDD_CMD gate_release, then re-arm"). The sounder's own retry (houdiniArmTdd, BaseRadioSet.cc:488-499: abort-only, expects an accepted=0 readback that no longer happens) can NEVER recover; the demo restarts today only because csi_server.py runs teardown_framer.py between attempts. FIX: run the full ladder (abort -> TX_CLEAR -> gate_release) in armHoudiniTdd and handle the throwing arm | experiment A2 x3 (refused+threw each time); A3 control x3 (ladder then arm accepted each time) | VERIFIED-HW (x3); FIX row OPEN |
| 3.3 | The demo's own "62" schedule trips the rx->tx frame-wrap abutment warning and sets the sched_abut_rx_tx sticky (this explains the sticky seen in every snapshot); the arm still runs. The warning concerns T/R-switch guard timing; bench impact presumed none but UNVERIFIED | TDD_SCHED + TDD_ARM warnings on every arm | VERIFIED-HW (occurrence); impact UNVERIFIED |
| 3.5 | BS RX stream under the slot-granular ring is timestamp-exact and loss-free at the app layer: every read carries HAS_TIME; continuity is tick-exact (gap=0 across all reads); stamp -> (frame, slot, sample) via the TDD_ARM epoch lands frame-reads at exactly slot=0 samp=0 after arithmetic alignment; 200 consecutive frame-reads advance frame by exactly +1 with 24,576,000/24,576,000 samples at ~1000 frames/s wall (real time, no backlog). This is the mapping the app layer should use for slot tags (replacing the energy-derived synthesis) | tests/demo-verify/bs_rx_timestamp_verify.py, runs 1+2 PASS, 2026-08-30 | VERIFIED-HW (x2) |
| 3.6 | RX samples flow BEFORE the epoch tick arrives (derived frame indices are negative pre-epoch): an idle/pre-epoch framer leaves the rx gate fail-OPEN. App code deriving (frame,slot) must handle delta < 0 | same runs, first-reads tables | VERIFIED-HW (x2) |
| 3.4 | The config frame_schedule is NOT programmed into the Houdini framer. armHoudiniTdd reads it only for indices (rx slots {16,18}, pilot 16) feeding the software energy-extraction; the hardware ring is derived from frame LENGTH alone ("62"). On Iris the same string WAS the hardware schedule (TDD_CONFIG translation, BaseRadioSet.cc:286-360, dead code here). Recorded rationales for the degenerate ring: (a) "pre-open guard would clip a slot-filling capture" -- UNVERIFIED folklore, check queued; (b) continuous-read extraction design -- circular; (c) free-running-clock era: the pilot walked the frame, so only a 0.5 ms window + energy search could catch it. Preconditions (c) and likely (a) are obsolete on the locked, grid-timed, MTS-capable stack -- the target model (slot-granular schedule translated from the config string) is the convergence point | BaseRadioSet.cc:505-599 (first-hand); UE_TX_FINE_GRID_TIMING.md tolerance record; TWO_BOARD_CLOCK_LOCK.md resolution | VERIFIED-CODE (mechanism); rationales flagged per item |

## 4. Beacon design audit + BS beacon HW verification

Schedule facts (answering: does the beacon take a slot, is it every frame,
is it TX RAM). References: BaseRadioSet.cc armHoudiniTdd (505-599),
buildHoudiniBeacon (391-436); driver refs spot-checked where cited.

| # | Claim | Evidence | Status |
|---|---|---|---|
| 4.1 | The beacon DOES own a hardware schedule entry: symbol 0 = '6' (replay_strobe + rx_gate) in the fixed 2-entry ring "62"; the ring is latched at arm and walks cyclically with no host writes, so the strobe pulse recurs once per ring pass = once per frame. What does NOT exist is a sounder-slot-granular symbol: the beacon's hardware symbol is 61440 ticks = 15 sounder slots wide, and loops=forever fills it (~15 RAM passes: 496-sample beacon + 3600 zeros each), so slot times 0..14 carry beacon copies and the schedule's G slots 1..15 are NOT silent on air. The per-frame recurrence (ring walk + per-pulse hardware re-arm) is independent of loops; loops only sets the within-window repetition | TDD_SCHED "62" + TDD_REPLAY_STROBE (BaseRadioSet.cc:553-587); strobe re-arm contract platform.h:640-655; recurrence measured: acked/frame=1.000000 (row 4.10) | VERIFIED-CODE + HW (recurrence); on-air copy count = phase 4 measurement |
| 4.2 | One beacon window per TDD frame = per 1 ms sounder frame, every frame, epoch-locked; first copy starts at window_open + 384 ticks (3.125 us) | spf_tdd = 122880/61440 = 2, one '6'/frame; offs=384 | VERIFIED-CODE; live +1 acked/frame check pending |
| 4.3 | The beacon is programmed as FPGA TX replay RAM: xmit on the tx_mode=replay stream -> writeRegisters("TX_RAM1") -> 4096-deep BRAM, ch1. len=2048 units = 4096 samples = full RAM, matching the load exactly. Repetition comes from loops=forever, not RAM content. The Iris BEACON_RAM path in BaseRadioSet.cc is dead code on this branch | buildHoudiniBeacon + xmit before strobe enable; host streaming.cpp:1334-1381; len units = 2-sample pairs (driver) | VERIFIED-CODE |
| 4.4 | The transmitted beacon has NO 128-sample prefix (buildHoudiniBeacon strips it; core at RAM head), while the UE frame-start arithmetic assumes beacon_start = slot_start + prefix (sync_index - 624). The constant offset (384-tick strobe offs, missing prefix) is absorbed by tx_advance calibration; the k x 4096 copy ambiguity is not | buildHoudiniBeacon:408-416 vs receiver.cc:1065-1066 | VERIFIED-CODE; consequences measured in phases 6-7 |

| 4.5 | TX replay RAM depth is exactly 4096 samples per channel (not 4095); the register fill path caps at that. RAM content for the demo = 496-sample beacon core + 3600 zeros | RFCORE_TX_RAM_DEPTH platform.h:702 (spot-checked); registers.cpp fill cap; buildHoudiniBeacon | VERIFIED-CODE |
| 4.6 | loops=forever replays until the window (symbol 0) closes; the close cleanly ENDS a started burst and acks it, so per-window acked is beacon liveness even in forever cadence. loops=N (plays=N) is supported, so a single-copy-per-frame beacon (loops=1) needs no schedule change | platform.h:640-655 (spot-checked); streaming.cpp:4258-4259 loops parse; :1222-1231 loop_cnt clamp | VERIFIED-CODE |
| 4.7 | Single-copy trade to measure: acquisition detect window (9543 samps) would contain the beacon in ~1 of 12.9 windows (slower initial lock); resync reads the slot-0 window per frame and finds the single copy where its arithmetic expects it | window sizes: receiver.cc:36,1056-1058; resync read receiver.cc:1152-1156 | VERIFIED-CODE (geometry); lock-time impact = phase 4 measurement |

| 4.8 | The beacon WAVEFORM is rate-agnostic: a 496-sample discrete sequence, no rate parameter in construction, no host upsampling; detection correlates the same sequence at the RX app rate, so any common TX/RX app rate works (duration and bandwidth scale with rate) | genPilots config.cc:756-813; buildHoudiniBeacon; find_beacon | VERIFIED-CODE |
| 4.9 | The sounder's Houdini TDD layer is HARD-LOCKED to 122.88 MSPS: it conflates samples with PL ticks (tick = 122.88 MHz always). spf_tdd = samps_per_frame/symbol_ticks mixes units (BaseRadioSet.cc:550-554); houdiniTddRx sums frame_ticks with samps_per_slot (:631-635); the 1 ms frame and the 320-grid-step identity hold only at 1 samp = 1 tick. Any other rate rung needs the samples/ticks conversions reworked and every ticks constant re-derived | code refs cited; HOUDINI_TICK_RATE=122880000 verified live both nodes | VERIFIED-CODE |

| 4.10 | With the sounder-fidelity preamble (rates + NCO + RX/TX-replay streams + RAM load), exactly one strobe burst per frame goes through the TX engine and PLAYS: acked/frame = 1.000000 and 1.000020, 1000.06-1000.08 acked/s over 50 s windows, late = gated = smiss = edge_late = 0. (Counters-level "plays"; on-air presence is the pending matched-filter step) | tdd_arm_experiment.py --preamble, runs 2+3 | VERIFIED-HW (x2) |
| 4.11 | Negative control (also validates the instrument in the failing direction): WITHOUT the preamble every burst goes LATE (acked == late, played = 0, smiss = 0) at the same 1/frame cadence -- the beacon arms but never plays. Which preamble element is load-bearing (rate apply at setupStream is the suspect) is NOT attributed | tdd_arm_experiment.py run 1 (no preamble) | VERIFIED-HW (x1); cause UNVERIFIED |

| 4.12 | TARGET-MODEL FEASIBILITY (arm + strobe level): the config schedule translated slot-for-slot DOES drive the hardware on the current stack. Ring "4" @0, "2" @16+18, "0" guards, symbol_ticks=4096, spf=30 arms cleanly (off-grid 4096-tick symbol width is accepted; epoch still 384-aligned), the frame-wrap abutment warning disappears (guards now separate rx from tx), and a single-copy strobe (len=248, loops=1) plays exactly one 496-sample beacon per 1 ms frame: acked/frame = 1.000033 and 1.000000, ~1000.1 acked/s, late = gated = smiss = edge_late = 0 | tdd_arm_experiment.py --sched 400000000000000020200000000000 --symbol-ticks 4096 --spf 30, runs 1+2, 2026-08-30 | VERIFIED-HW (x2); RX-side gated-capture semantics + on-air check still open |

| 4.13 | TX amplitudes in int16 DAC counts (the ONLY level control; gains are no-ops): beacon peak 19660 (60% FS, BaseRadioSet.cc:423); pilot peak ~8192 (25% FS: tx_scale=1/(4 x max_amp) then x32768, config.cc:868-875 + utils.cc:53); UE data same scale region; guards 0. The "6dB Power backoff" comment is WRONG: /4 amplitude = -12 dB. cfloat_to_cint16 multiplies by 32768 with a truncating cast (float 1.0 would wrap; safe at 0.25 FS). int16-to-14-bit-DAC bit alignment in the gateware: NOT verified | cited lines spot-checked 2026-08-30 | VERIFIED-CODE (amplitudes); DAC bit alignment UNVERIFIED |
| 4.14 | Detectability of the single-copy beacon: per-copy statistics unchanged (same 496-sample core; threshold is LOCAL trailing energy). Measured populations on this link: beacon windows ratio 0.31-4.2 (median 2.8) vs empty 3.4e-7; acquisition bar 0.1 -> >=3x margin. Cost is lock latency only: fresh-window hit fraction ~= (9543-496)/122880 = 7.4%. No per-rate resampling needed while both nodes share one app rate (4.8); regenerate per rate only if constant duration/bandwidth across rates is ever required | 9ee0567 measurement record; geometry | VERIFIED-CODE + prior measurement; lock-latency number = pending live |
| 4.15 | Target schedule shape [user 2026-08-30]: 1 TX slot (beacon) + 2 RX slots (P,U) + 2-4 margin RX slots (debug-only drift catch), guards closed. Blocked on the driver capability (hardware-chained windowed RX; gate close abandons a continuous capture, host pump ~100 ms/window). Interim shipped (c64a557): slot-granular ring, single-copy beacon, all-rx gating. Wire cost: interim ~492 MB/s vs target ~82 MB/s BS ingest | rx-probe evidence (zero data gated vs streaming control); D4 contract | SPEC recorded; [-> propose SH ticket] row to file |

Beacon-arrival campaign findings (2026-08-30, instruments:
tests/demo-verify/two_node_beacon_arrival.py + map_scan.py):

| # | Claim | Evidence | Status |
|---|---|---|---|
| 4.16 | FINAL CABLE MAP (verified, symmetric, matching [user] intent): self-loopback TX0->RX0 on BOTH boards; two-node link BIDIRECTIONAL on channel 1 (= "B") both ends: .21 TX1 -> .22 RX1 (rms 238, ratio 1.7e9) AND .22 TX1 -> .21 RX1 (rms 284, ratio 5.6e9). topology-houdini.json roles and the config's B/B are CORRECT AS-IS. Two earlier wrong readings recorded for the audit trail: "bench not in demo cabling" and "roles reversed" -- both artifacts of the RF-dead arming state (4.24) in pre-instrument scans; corrected by [user] pushback + re-measurement | map_scan.py, all four orientations, with liveness counters | VERIFIED-HW |
| 4.17 | The channel map did NOT move across sessions this campaign (TX0->RX0 hot in both .22 scans, hours apart); the dynamic-map concern remains a per-boot question, re-verify after any reboot | map_scan x2 | VERIFIED-HW (this boot) |
| 4.18 | The C++ acquisition rule (earliest crossing refined within 128) can anchor INSIDE the STS region: 16-periodic STS is self-coherent at lag 128 and crosses the threshold ~430 samples before the gold 2-rep peak, with ~+-30 samples of scatter between detector contexts. Constant bias is absorbed by tx_advance; the SCATTER is per-lock-decided. A gold-only beacon (the 4.x audit candidate) removes the ambiguity class | measured on .22 loopback: 13 consecutive confirm residuals -430+-30 independent of frame gap k (1..562) | VERIFIED-HW (instrument context); C++-path impact plausible, not yet measured in the sounder itself |
| 4.19 | Session-structure effect on the TX path, cause UNKNOWN: arming from a TX-only session plays the beacon STRONG on the cabled path; arming from a session that also holds an RX stream measured only crosstalk twice (RX activated before or after the arm made no difference). Row 4.11's "no preamble -> all late" re-attributes: the TX setupStream (rate apply + sync) is the load-bearing element, NOT the RX stream | shared-session runs b (weak x2) vs map_scan sessions (strong x2); liveness counters played healthy in ALL cases -- counters do not prove RF | VERIFIED-HW (occurrence x2 each way); mechanism UNVERIFIED, driver-lane question |
| 4.20 | SoapySDRServer wedges after some client sessions: next make() gets connections accepted but never serviced ("handlerLoop FAIL: SoapyRPCUnpacker::recv(header)"), previous session's teardown fully clean in the journal. 3 occurrences today (.21 x1, .22 x2); recovery = systemctl restart (passwordless sudo on boards). [-> propose SH ticket] with the journal capture | .21 journal 18:06-18:08; occurrences logged | VERIFIED-HW (occurrence x3) |

| 4.21 | BEACON ARRIVAL IS SAMPLE-EXACT ON AIR: with a genuine anchor (real-beacon windows score ratio >=1e7; an absolute floor rejects noise-window artifacts), every tracked arrival landed at exactly the predicted tick: two-node 60/60 and 40/40, loopback 40/40 -- residual mean/std 0.00, drift 0.000000 samp/frame over 93-140-frame spans | two_node_beacon_arrival.py money runs (json on rig) | VERIFIED-HW (x3 runs) |
| 4.22 | Framer re-arm determinism ON AIR: across re-arms the arrival-phase shift equals the epoch delta EXACTLY (err=0 samples, x2, two-node). One loopback re-arm attempt failed to reacquire (beacon apparently silent post-re-arm) -- re-arm RF reliability is not yet 100%, same state-sensitivity class as 4.24 | money run M3; loopback M3 failure | VERIFIED-HW (determinism when alive); reliability flake logged |
| 4.23 | NO-MTS BRING-UP VARIABILITY (answers 2.18): across 3 full device re-makes (complete tile shutdown/startup), beacon arrival phase moved -1, -4, -5 samples from epoch-predicted (method: same-board loopback, both epoch and stamps on one clock axis so origin resets cancel). Real, tiny (<=41 ns); a per-restart-decided constant. Two-node M4 is INVALID by design: make() RESETS the board time origin (observed: post-remake epochs collapse ~258M -> ~46M ticks) | arrival_loopback_M4.json | VERIFIED-HW (n=3) |
| 4.24 | PRE-ARM MUST BE THE FULL LADDER: abort-only pre-arm produced a beacon that PLAYED in the counters but emitted NO RF (twice); full ladder (abort -> TX_CLEAR -> gate_release, + strobes off) before arming emitted strongly (twice). TX_CLEAR is the load-bearing step ("a consumed arm parks the source until the arm clears or TX_CLEAR"). UPGRADES 3.2: armHoudiniTdd's abort-only is not just a re-arm-throw risk, it is an RF-dead risk; the demo has been protected only by teardown_framer.py running the ladder between attempts. FIX row for the sounder | weak/strong differential runs on .22 TX1, no server restarts between | VERIFIED-HW (x2 each way); exact mechanism = driver-lane question |
| 4.26 | ROLE-SWITCH SYMMETRY: the full arrival measurement run in BOTH directions is sample-exact and equivalent -- demo direction .21->.22: lock 22 windows, 60/60 resid 0/0/0 drift 0.000000 over 134 frames, rearm err=0; reverse .22->.21: same zeros (4.21/4.22). Also proves .21 TX1 hardware is fine; every earlier ".21 dead" observation = the 4.24 arming state | arrival_demo_direction.json + money run | VERIFIED-HW |
| 4.27 | RFDC interrupt rate ~1 per frame (~1.1 kHz) during strobe operation: one session logged 228,895 interrupt firings at teardown -- a DAC FIFO margin event PER BURST is the likely source (and explains the latched FIFO_OVR/UF warning at every next construction). CPU cost + meaning = driver-lane question | IRQ teardown counts in scan logs | VERIFIED-HW (occurrence); attribution UNVERIFIED |
| 4.28 | Beacon SNR at the UE: core-only rms 1183 counts (.21->.22) / 1414 (.22->.21) vs noise rms ~6.1 -> 45.8 / 47.3 dB per-sample (core at 3.6-4.3% ADC FS), plus ~21 dB detector correlation gain. Measured beacon position in slot 0 (same-board clock, full analog chain): core starts at slot_start + 504..509 across 4 bring-ups = 384 (strobe offs) + ~122 samples pipeline latency, stable +-2. The UE's slot-grid model constant (sync_index - 624, assuming beacon at slot_start+128) differs from this truth by a FIXED offset -- exactly what tx_advance / ue_tx_advance_ticks calibrate; alignment is relative + one constant, never absolute, and the stability of that constant is the measured part | window rms sets; (anchor-epoch) mod frame from loopback jsons; receiver.cc:1065-1066 | VERIFIED-HW + CODE |
| 4.29 | THE OFFSET BUDGET, all three layers on the record: BS model = core commanded [384, 880) in slot 0 (offs to offs+beacon_size); measured = [~506, ~1002) (+~122 pipeline latency, +-2 across bring-ups); UE model WAS [128, 624) (the Iris [prefix][beacon] layout) -> 256 samples of pure model error in the UE slot grid, silently absorbed by tx_advance. FIXED: receiver.cc now derives the anchor from the transmitted layout (houdiniBeaconEnd = 384 + beacon_size for houdini) at both the acquisition and resync sites. CONSEQUENCE: tx_advance (135, tuned under the old grid) now over-advances by ~256 and MUST be re-derived from the phase-7 pilot-arrival measurement; until then the BS energy-align absorbs the shift | receiver.cc houdiniBeaconEnd + call sites; rows 4.16/4.28 | VERIFIED-CODE (fix) + HW (budget); tx_advance recal OPEN |
| 4.30 | END-TO-END INTEGRATION SMOKE (all changes together): full demo ran to max_frame 600 and exited rc=0 on the rig build @ 6b19ee5. Slot-granular ring armed via the ladder first try; UE locked the single-copy beacon with the 2-detect rule and the corrected grid constant (offset -510 = sync_index 370 - 880); BS captured the pilot EVERY frame at pilot-rms 1385 (+6 dB from the half-FS TX scale change, as predicted), selfsim 0.98, monotonic frame counter, no version skew, no BAD writes | rig logs/smoke_e2e_1.log, 2026-08-30 | VERIFIED-HW (x1; repeat + score_runs A/B next) |
| 4.31 | THE BS NOW REDERIVES THE UE SCHEDULE PER FRAME from the pilot arrival (HOUDINI_BS_RX_DEBUG: pilot_grid_off, pu_spacing_err). First run: both CONSTANT across 220 frames -> the UE's realized schedule is sample-stable within a run. pu_spacing_err = +255 ~= the PREDICTED +256 grid-snap branch (P/U snap independently to the 384-tick grid; per-run draw is -128 or +256) -- the AP-15 correlation test is now one log line + score_runs. OPEN: pilot_grid_off = -10683 (~2.6 slots early vs the nominal ~+100 budget) -- a real, constant UE-anchor discrepancy the energy-search had been hiding; decompose in phase 6/7 (detector index semantics, corr_scale artifact locks, stamp-vs-count anchor arithmetic under drain) | rig logs/smoke_e2e_2.log @ 6f2e707 | VERIFIED-HW (stability + snap prediction); absolute offset decomposition OPEN |
| 4.25 | Noise-window detector behavior differs from the recorded C++ populations: windows with NO burst score ratio ~300-900 in the python replica (historic C++ bench: 3.4e-07). Either replica normalization differs or a lag-128-coherent artifact exists at noise level. Instruments now use an absolute ratio floor (1e4); the C++ acquisition threshold semantics at noise level should be re-derived before trusting corr_scale bars in no-signal regimes | hunt logs across all runs | VERIFIED-HW (occurrence); cause UNVERIFIED |

## 5. UE device init

(filled in phase 5)

## 6. UE beacon acquisition

(filled in phase 6)

## 7. UE timed TX + BS gated RX

(filled in phase 7)

## Standing traps (carried from the driver contract, apply to every phase)

1. An unknown or non-writable writeSetting key logs a warning and silently
   no-ops; an unknown read returns "". Never trust absence of an exception;
   always read state back. (device/SoapyHoudiniSDR_streaming.cpp:2102-2117)
2. Teardown ladder is abort -> TX_CLEAR -> gate_release. The sounder issues
   only abort. (section 3 investigates the consequence)
3. TX_RAM fill is refused while the channel is level-armed or strobe-enabled.
   len units for TDD_REPLAY_STROBE are 2-sample pairs.
4. rx_gap_break is silently disabled for a stream's lifetime if the rate
   probe at setupStream fails (loud warning only).
5. The version skew check warns and never aborts; grep logs for VERSION SKEW.
6. Client pthreads are never joined; teardown under SIGINT is racy.
