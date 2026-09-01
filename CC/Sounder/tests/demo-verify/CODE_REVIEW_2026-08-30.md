# Opus code review, 2026-08-30: feat/csi-gui-tabler, range 1ae17ad..HEAD

Verbatim findings from the independent Opus review of the day's ~70 commits.
Disposition of every item is recorded in DEMO_VERIFICATION.md row 4.56; the
burn-down landed in the commits immediately following this file's addition.
Severity: HIGH = wrong behavior possible on this bench, MED = latent/edge,
LOW = hygiene.

## HIGH

- H1 BaseRadioSet arm retry: a refused TDD_ARM re-ran the teardown ladder then
  retried the arm WITHOUT re-loading the replay RAM / TDD_SCHED / strobe.
  A retry that succeeded could arm a beaconless framer that logs healthy.
  FIXED: the load/sched/strobe sequence is a re-runnable step invoked after
  every ladder in the retry loop.
- H2 tests/rx-recorder/build (61 files, 5.56 MB, four ELF binaries) was still
  tracked. FIXED: untracked before any push.
- H3 landing_map snr_db omitted the live metric's 8-sample guard band, so the
  replica re-measured the cliff the guard removed. FIXED.
- H4 landing_map read_meta truncated fractional values (snr 31.8 -> 31); the
  truncated numbers reached ledger 4.42. FIXED; ledger annotated.
- H5 landing_map onset/end carried the 64-tap envelope's ~29-sample bias and
  compared raw numbers against the nominal layout. FIXED: raw-power edge
  refinement + cross-check against the C++ p_start/u_start in the same file.
- H6 evm_compare decision-directed fallback lacked the -pi in the 4th-power
  derotation: reported SNR pinned at 2-4 dB anti-correlated with truth. FIXED.
- H7 evm_compare ramp search reached ~+-2.5 samples vs the sounder's +-8 and
  the measured +3 draws; saturated edge reported as an answer. FIXED: +-8
  coarse grid, no-lock guard, edge warning, sounder sign convention.
- H8 ap15_correlate scored one average over the whole run's pooled points --
  a 5% garbage-frame class still scored 0.949 "CLUSTERS". FIXED: per-datagram
  scores; verdict = median + worst + low fraction (CLUSTERS needs low<=0.5%).
  Ledger 4.41 annotated: the AP-15 closure stands on 4.53's per-datagram
  0/1024 soak, not on the old run-average.
- H9 fake_feed filled the CSI2 trailing block with [0,1] coherence which the
  page now draws as raw phase. FIXED: emits the instrumental sawtooth.

## MED (production)

- M1 houdini_pilot_cursor_shift_ was never written anywhere. FIXED: removed;
  the comment now matches the one-mode (reset) contract.
- M2 escalation reset re-queued ~horizon frames over the already-queued
  bursts. FIXED: the cursor resumes on the NEW grid at the first slot after
  everything queued; the stale-grid tail drains behind it.
- M3 kLead(700) < kScatterTol(1024): residuals in [-1024,-444] undetectable.
  FIXED: kLead=1280.
- M4 escalation-probability comment described the pre-targeting search.
  FIXED: rewritten for targeted semantics.
- M5 one reset flag shared across client threads. FIXED: per-tid flags.
- M6 ue_tx_advance_ticks silently quantized by the 3125 ns snap. FIXED:
  one-time warn + both config notes rewritten.
- M7 quiet path delivered duplicate-(frame,slot) noise packets for the length
  of a UE pause. FIXED: returns 0; loopRecv's no-slot path releases cleanly.
- M8 untrusted-frame marking was view-only; recordings kept the poison
  silently. FIXED: LTS-check refusals now push a cause-5 extent into
  /Data/Gaps.
- M9 CNS score gate admitted 16/64-QAM against a QPSK-calibrated threshold.
  FIXED: QPSK only.
- M10 stage-2 refinement had no deep-fade gate; loop-invariants recomputed;
  break-vs-continue bug. FIXED.
- M11 phase anchor drawn from the first datagram with no settle. FIXED:
  third-update settle gate (never re-anchors mid-run, by design).
- M12 Radio ctor leaked the aux MTS stream + device on a setupStream throw,
  undermining the in-process retry. FIXED: cleanup-and-rethrow.
- M13 config notes said 20 dB floor (code: 30) and described the snap-era
  model. FIXED.

## MED (GUI/cross-consumer)

- M14 setScAxis labeled the phase panels instead of the waterfall after the
  stack landed. FIXED: labels all four subcarrier gutters.
- M15 score_runs read the retired qual_med and printed nan. FIXED: column
  removed.
- M16 drawAdc hardcoded 128-sample guard markers (eight shipped configs use
  160). FIXED: templated from the config like the magnitude axis.
- M17 CSI2 docstrings still described the coherence block. FIXED both sides.
- M18 ADC_FS hardcoded a third full-scale copy. FIXED: drawAdc reads the
  record's full_scale; honest container-vs-converter comment (absolute
  mapping remains unmeasured, ledger 2.19).

## MED (instruments) and LOW

All remaining instrument MEDs fixed in the same pass: mode-B peak-to-sidelobe
gate, core length from the file, summary occurrence counts, signed onset
fold, read_iq guards, err/sig on the reference-valid mask, aggregate pilot
SNR, reference-tag warning, burst-count sanity warning, CP-plateau honest
labels, ap15 settled-draw selection with drift check, kMaxPts comment,
truncated-datagram guard, logs dir creation, ue_init_walk rc/channel/scope
fixes. LOW hygiene: dead sticky-axis machinery, quality CSS/colors, stale
MAGIC/docstring text, pacc removal, deque include, r-sweep hoist, member
privacy, comment corrections (utils saturation, kAdcCols, field widths,
frame-multiple claim, dump header), CNS autopsy filename carries ant id,
full-scale print consistency, nan-proof axis args.

Accepted as-is, documented instead of changed: /tmp paths for the two gold
dumps; hardcoded instrument geometry (noted in docstrings); four copies of
the 384-tick grid constant (cross-referenced at the BaseRadioSet definition).

## Reviewer's verified-correct list (kept for the record)

De-ramp sign; composed-burst bounds and odd-total handling; tail-copy re-map
bound arithmetic (fires for at > 126976, earlier copy always exists);
radioTx's snap is a no-op for grid-exact anchors (122880000/384 = 320000);
n_bs_sdrs restore consistency + dtor vector iteration; tx_advance default
ordering; every wire format byte-for-byte against its parser; LTS_F identical
to Consts::lts_seq; csi_h_hist_ removal complete.

## Overall

"The production C++ is sound and the day's engineering is real... The
instrument layer is where this branch is weak" -- hence the same-session
burn-down above and the per-datagram re-basing of the AP-15 closure evidence.
Counts: 9 HIGH, 32 MED, ~30 LOW.


# Second review (verification pass over the burn-down)

Counts: 27 fixes confirmed (several verified mechanically: the r-sweep hoist
proven algebraically exact, the SNR replica proven at exact parity with the
live metric, the ceil resume arithmetic checked on every boundary, the wire
formats byte-for-byte). 4 defective-or-partial and 12 new findings, all
fixed in the follow-up commit:

- 2.1 HIGH: rxs_/txs_ had no initializer, so M12's cleanup could closeStream
  a wild pointer on the board-wedge retry path. Fixed: nullptr NSDMI.
- 3.1: three unbraced multi-statement MLPD_INFO throttles flooded logs at
  full rate (the CSI timing-fix line at ~1 kHz under the runbook's own demo
  launch). Fixed: braced.
- 2.2 long-long format UB in the new config warn; 2.3 evm edge warning now
  tracks the active search span; 2.4 the no-slot warning no longer
  misdiagnoses a UE pause; 2.5 axis validation moved before the sounder
  launch (a late SystemExit orphaned the radio-holding group); 2.6 the
  full_scale comment no longer overclaims (the parser is the single
  page-side source; the wire does not carry it); 2.7/3.2/3.3 walkthrough
  drift (third-update anchor, real --mag defaults, config-driven guard
  wording); 2.8 fake_feed retired options pruned; 2.9 dead ternary + ramp
  sign; 2.10 targeted-resync occupancy figure updated for kLead=1280;
  M6/M13 completed in houdini-1u.json (the "sweep it" advice and the
  whole-ms claim retracted); the no-op const_cast dropped; the cause-5 enum
  comment now states the extent is the pilot slot with an approximate start.

Residual accepted items (documented, not changed): the settle-gate's two
unanchored first updates; ap15's zero-tolerance gate under 200 datagrams
(conservative, now commented); the k>horizon reset corner the reviewer rated
effectively unreachable.

Second reviewer's verdict: "the signal-path engineering is sound and
independently checks out... two one-line fixes away" -- both landed.
