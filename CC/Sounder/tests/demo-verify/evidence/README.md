# Evidence captures

The ledger (`CC/Sounder/DEMO_VERIFICATION.md`) cites files under this
directory by path. Small artefacts (campaign logs, gate summaries, startup
blocks, per-run logs) stay here.

The raw sample captures of 2026-09-02 (`20260902-rig/*.json`,
`20260902-rig2/*.json`, sixteen files, about 251 000 lines) left the tip of
the branch at landing, per the packaging decision in
`docs/SOUNDER_CHANGE_PACKAGING.md` section 5. They remain in history: the
last commit that carries them is `ab408f5`, so a cited path resolves with

    git show ab408f5:CC/Sounder/tests/demo-verify/evidence/<file>

Nothing in the tests or tools reads them; the golden windows the tests
replay live in `tests/comms-func/fixtures/golden` and are unaffected.
