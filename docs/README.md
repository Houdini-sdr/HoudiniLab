# Design notes

The reasoning behind the sounder's shape, kept where the code can cite it.
Procedures live in `CC/Sounder/CSI_DEMO_WALKTHROUGH.md` and
`CC/Sounder/DEMO_BENCH_RUNBOOK.md`; evidence in
`CC/Sounder/DEMO_VERIFICATION.md`; finished investigations under `archive/`.

| Note | Question it answers | Where it stands |
|---|---|---|
| `RADIO_PLATFORM_SEAM.md` | How the sounder talks to a radio, so that the Houdini, Soapy and UHD backends share one interface and one set class per role | Its first three steps are on `develop` (the interface, the factory, the set classes); the build matrix across backends and the last step are open |
| `SYNC_LIBRARY_ARCHITECTURE.md` | How the client's beacon detection, confirmation, carrier estimate and grid tracking become one configured, tested library | The library exists (`CC/Sounder/sync/`) and its detection bar is derived from a false-alarm target; the phase tracker it plans remains open. Its first section describes the environment-variable knobs as they were: they are gone, and the config's `sync` block (walkthrough section 7.1) is the one knob path |
| `SOUNDER_CHANGE_PACKAGING.md` | Whether the Houdini work left RENEWLab's sounder intact for Iris hardware, and how to package the fixes apart from the Houdini features | A proposal, with the decisions it leaves to the project's owner listed in its last section; the Iris paths were verified intact by inspection, not on Iris hardware |
| `../CC/Sounder/PER_BAND_BW_DESIGN.md` | How each band carries its own width on one numerology (270 resource blocks on X-band beside 133 on sub-6) | Implemented and in every dual-band configuration; its rig predictions were run and the record says which held |
| `archive/` | Finished investigations: the two-board clock lock, the client's fine-grid transmit timing, the time and frequency sync campaign | Their results live in the walkthrough and the configs; `archive/README.md` says where |
