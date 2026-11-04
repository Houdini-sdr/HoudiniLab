# When something goes wrong

Each known failure, by its symptom: what you see, what it means, what to do. Section 8 of `CSI_DEMO_WALKTHROUGH.md` is the index into this file; when in doubt, run the setup check first (its section 2.2), since every FAIL line names its fix.

## 1. The run produces almost no output, or says it cannot find a radio

This is the most common failure on a busy bench and it is almost never the
radio. Look for a line like this early in the log:

```
setSampleRate(RX): an RX stream is open; a rate change is NOT live
```

A previous run left a receive stream open on the board. The board will refuse
every new run until that stream is released, and a refused run writes almost
nothing to its log, so it looks like nothing happened rather than like an error.
If you are running several captures in a row, the first one that fails this way
makes all the rest fail too.

The usual cause is how the previous run ended. A sounder started by the
launcher ends when the launcher ends, however the launcher was killed, and
a sounder you started yourself releases both boards when you stop it. What can
still hold a board is a sounder started some other way (another window, another
user, a test harness) that is still running.

Release it, from `<host>`, in the sounder directory:

```sh
python3 tools/rig_release_holders.py
python3 csi_gui/teardown_framer.py --conf files/<config>.json   # or --topology <its serial_file>
```

The first command stops EVERY sounder, dashboard backend and teardown on
`<host>`, including a dashboard someone else left running, so ask first on a
shared host. The second confirms the boards are clear; it prints
`all 2 radio(s) clear` when they are (section 5 explains the topology
argument). Then start your run again.

If the release tool finds nothing and the boards are still held, something
outside your session is holding them, and the board's own server has to be
restarted, on each held board:

```sh
ssh <user>@<radio-ip> 'sudo systemctl restart SoapySDRServer'
```

That needs a password, so on a shared bench it is worth asking whether a
colleague is using the boards before reaching for it.

## 2. The dashboard shows "connecting" and never populates

The page is reaching the backend but no datagrams have arrived. Confirm the
sounder printed its `CSI view mode` line, then confirm the backend datagram
count is climbing (`CSI_DEMO_WALKTHROUGH.md` section 6). If the sounder never printed that line, it is
not in viewing mode: check that you passed `--view`, or `--csi_udp`.

## 3. Panels appear but the waterfall tears horizontally

The client is losing and re-acquiring the beacon. Confirm from the log before
guessing: escalations print `Re-sync ESCALATION` with a reason, and the base
station prints `UE PILOT LOST` during the outage. Usual causes, most likely
first: the clock plan (`CSI_DEMO_WALKTHROUGH.md` section 2.1: no shared reference and no steering, or a
board out of its calibrated hold, which the full setup check's `clock` line
reports), the RF level is wrong so detections fall under the sync SNR floor
(the re-sync lines print the measured SNR; compare it against the config's
`sync.confirm.snr_floor_db`), or the detection bar (`corr_scale`) needs
adjusting for your path.

## 4. Channel estimate looks fine but the constellation is a smear

Confirm the config's frame carries an uplink data slot (`U` in the schedule:
`houdini-ul.json` or any dual-band config); without one there is nothing to
equalize. If it does, work through the three causes below in order.

**Cause 1: the FFT window is sitting late and taking in interference.** Try
the sounder's `--csi_sym_start` a few samples lower and watch the constellation
tighten. `SOUNDER_REFERENCE.md` explains the asymmetry: early is recoverable, late is not.

**Cause 2: dropped receive packets.** When a receive packet is lost,
`RadioHoudini::recv` zero-pads the hole so the rest of the window keeps its
true timing. Those zeros are not signal, and an FFT taken across them produces
a wrong channel estimate, which would then equalize every following uplink
data slot until the next clean pilot.

Viewing mode therefore refuses those slots instead of rendering them. When a slot
arrives carrying padded samples it is dropped, that antenna's card dims with a
`stale` badge (`DASHBOARD_GUIDE.md` section 2), and the sounder logs, at most once every five
seconds:

```
CSI view: dropped 12 slot(s) with RX gaps (latest 848 padded samples, ant 0)
```

So the display holds its last good estimate rather than showing a false one,
and marks it as old rather than passing it off as current. **A dimmed card plus
that warning means the link is losing packets**, and the fix is on the link, not
in the viewer. Recording mode keeps every sample and records the damaged
ranges in the file's gap table (`/Data/Gaps`), so a recording is not evidence
of a clean link until its gap table is read. To confirm the loss rate, record
a capture over the same link and compare its gap table against how often the
warning appears.

**Cause 3: a failed transmission at the client.** The pilot and the uplink
data ride one composed burst per frame, and its transmit return is checked:
watch the sounder log for `BAD Write` and `unexpected writeStream error`. The
client also drains the driver's asynchronous transmit status once per queue
top-up, so a burst that was accepted but later reported late or dropped is
surfaced rather than lost.

If the constellation is a ring or smears differently from one restart to the
next with none of the above logged: that class of fault (timing offsets
between the pilot and data paths) is fixed (`DEMO_VERIFICATION.md` rows 4.36
to 4.48). A persistent smear points at RF level, clipping, receive gaps or,
on the dual-band configs, the carrier offset (`CSI_DEMO_WALKTHROUGH.md` section 2.1), not at restart
luck.

## 5. The sounder will not start, and discovery looks broken

A run that was killed rather than stopped can leave the base station framer
armed and the transmit RAM loaded. The next run then fails to start, and it
usually looks like a discovery or network problem rather than leftover state.

Clear it:

```sh
cd <path-to-HoudiniLab>/CC/Sounder
grep serial_file files/<config>.json          # the topology your config runs
python3 csi_gui/teardown_framer.py --topology <that file>
```

It reads the radio addresses from the topology file, opens each one, issues
the framer abort, clears the transmit RAM, and releases the gate. Name the
radios one of three ways: `--conf files/<config>.json` (the topology that
config's `serial_file` names), `--topology <file>`, or `--node <addr>`
(repeatable). With none of them it refuses rather than guess. With the
plugin in a prefix of its own (`CSI_DEMO_WALKTHROUGH.md` section 2.4), add `--soapy-root
<host-plugin-prefix>` to every teardown command in this document.

Read the exit status, not just the output. It is 0 only when every radio was
cleared, and non-zero when one could not be opened or torn down, which is
normally the actual reason the sounder then fails. `--launch` runs this for you
before each attempt and prefixes its output `[teardown]`.

This script opens a connection to each radio, so it is a device-touching
operation. Do not run it against boards someone else is using.

If it cannot import `houdini_setup`, pass
`--examples <path-to-SoapyHoudiniSDR>/host/examples` (`CSI_DEMO_WALKTHROUGH.md` section 2.4).

If the teardown itself reports that an RX stream is still open, a dead process
left one behind and no teardown can close another process's stream. Restart the
server on that node and try again:

```sh
ssh <user>@<radio-ip> 'sudo systemctl restart SoapySDRServer'
```

That is also the recovery when a radio open fails with
`SoapyRPCUnpacker::recv() TIMEOUT`.

## 6. A radio is discoverable but still will not open

This one costs the most time if you do not know it, because the tool that
reassures you is testing the wrong thing.

**The two planes are independent.** `SoapySDRUtil --find` talks to the radio's
control plane over TCP. Streaming uses a separate data-plane network, and the
host needs a route to the address the radio advertises there. A radio can list
perfectly in `--find` and still fail to open, and the error you get names the
open failure, not the routing:

```
Ignoring houdini radio <addr>: FindLocalAddrForRemote: no interface routes to <data-ip>
ERROR: the above base station radio(s) could not be opened.
```

Check the route rather than the discovery:

```sh
ip route get <the data-plane IP from the error>   # want a real data interface
ip -br addr                                       # is that interface UP with an address?
cat /sys/class/net/<iface>/carrier                # 1 = cable and link partner present
```

`NO-CARRIER` means the data cable is not plugged in or the far end is down. An
interface with no address on that subnet means it needs one; the two radios may
sit on different data subnets and need one host port each.

**Do not use ping to test this.** The radios do not answer ICMP on their
data-plane addresses even while streaming perfectly, because that address is a
raw UDP egress engine in the FPGA, not a full IP stack. A failed ping proves
nothing here. The route existing is the check that matters.

## 7. Client never finds the beacon

Check in this order: the clock plan (`CSI_DEMO_WALKTHROUGH.md` section 2.1), the RF path is actually
connected and at a sane level, `frequency` and `nco_frequency` match each other
in the config, and the beacon board is really the one named under
`BaseStations` in the topology file.

Acquisition requires more than a correlation peak: the detection must clear
the sync SNR floor (`sync.confirm.snr_floor_db`: 30 dB of true link SNR by
default, 25 dB in the dual-band configs for their beacon) and then repeat
twice at exactly one frame spacing before the client anchors. A
marginal RF path can therefore correlate occasionally yet never acquire. The
re-sync and acquisition log lines print the measured SNR; on a cabled bench
expect the mid 40s dB, and treat much less as an RF level or cabling problem
rather than a software one.
