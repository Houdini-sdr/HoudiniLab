# Running the live CSI dashboard demo, step by step

This walkthrough goes from an empty machine to a live channel display in a web
browser: install the dependencies, build the sounder, point it at your own two
radios, run the base station and client together, and read the four panels the
dashboard draws. Every step shows the exact command and what you should see.

Five placeholders appear throughout. Add your own values:

- `<bs-ip>`: the address of the radio node you will run as the base station
  (the RFSoC running the Houdini server).
- `<ue-ip>`: the address of the radio node you will run as the client (UE).
- `<host>`: the machine that runs the sounder and the dashboard. It needs
  network reach to both radios. This is normally your compute host, not a
  radio.
- `<path-to-HoudiniLab>`: wherever you cloned this repository on `<host>`.
- `<your-houdini-venv>`: the virtual environment prefix where the
  SoapyHoudiniSDR host plugin is installed.

Additional reference material lives in `../../docs/UE_TX_FINE_GRID_TIMING.md`
(why the client pilot lands on the fine timing grid) and
`../../docs/TWO_BOARD_CLOCK_LOCK.md` (why both boards must share one reference
clock).

## 1. What the demo does

- The sounder normally records to HDF5. In **viewing mode** it does not write a
  file. Instead it computes a channel estimate from every received pilot and
  streams the result out as UDP datagrams, one per frame per antenna.
- A small Python backend, `csi_gui/csi_server.py`, receives those datagrams and
  serves a self contained web page. The page uses Server Sent Events and an
  HTML5 canvas, with no external JavaScript libraries.
- The page is styled with the Tabler theme, so it matches the RayNet compiler
  dashboard. Tabler ships as one stylesheet inside this repo at
  `csi_gui/vendor/tabler.min.css`, and the backend serves it. Nothing is
  fetched from the internet and nothing has to be installed, so the dashboard
  still works on a host with no network access.
- Two radios take part. One is the base station: it transmits a beacon and
  opens a receive window on each pilot slot. The other is the client: it locks
  onto the beacon and transmits a pilot back inside that window.
- The channel estimate is pilot agnostic. It correlates against the frequency
  domain reference built from your config, so an LTS, a Zadoff Chu, or any
  other `pilot_seq` works without code changes.
- The dashboard draws one card per receive antenna and scales automatically to
  however many antennas appear in the stream.

## 2. Install everything (from nothing)

### 2.1 What you need before starting

- A compute host `<host>`: a Linux machine (x86_64 or aarch64) with network
  reach to both radios.
- Two Houdini radio nodes at `<bs-ip>` and `<ue-ip>`: powered, running the
  Houdini server, on a firmware stack your team has blessed. Board bring up is
  owned by the SoapyHoudiniSDR and Houdini-Streaming projects and is not
  covered here. If you did not set the boards up yourself, ask whoever did.
- **Both boards driven from one reference clock.** This is not optional. Two
  free running boards drift by roughly 894 ppm, which walks the client pilot
  across the whole base station frame in about a second, and the pilot never
  stays inside the receive window. Feed both boards a common 10 MHz on `CLK IN`
  and confirm the firmware selects the external mux. See
  `../../docs/TWO_BOARD_CLOCK_LOCK.md` for the evidence and the verification
  procedure.
- An RF path between the two boards, cabled or over the air, at the frequency
  your config names.

### 2.2 Shortcut if your host is already provisioned

Verify with the four checks below and, if they all pass, jump straight to
section 3:

```sh
ls <path-to-HoudiniLab>/CC/Sounder/build/sounder      # binary exists
SoapySDRUtil --info                                   # plugin path is set
python3 -c "import http.server, socket, struct"       # dashboard needs only stdlib
ls <path-to-HoudiniLab>/CC/Sounder/csi_gui/vendor/tabler.min.css  # page stylesheet
cat <path-to-HoudiniLab>/CC/Sounder/files/topology-houdini.json   # your two IPs
```

### 2.3 System packages

On Ubuntu (22.04 or 24.04), install the build tools and libraries:

```sh
sudo apt install build-essential cmake git \
    libgflags-dev libhdf5-dev python3
```

What each is for:

- `build-essential`, `cmake`, `git`: compiler and build system.
- `libgflags-dev`: command line flag parsing.
- `libhdf5-dev`: HDF5, version 1.10 or newer. Viewing mode does not write a
  file, but the sounder links HDF5 unconditionally, so the build needs it.
- `python3`: the dashboard backend. It uses only the standard library, so
  there is nothing to pip install for it.

### 2.4 SoapySDR and the Houdini driver plugin

The sounder talks to the radios through SoapySDR and the Houdini host plugin.
Install the SoapyHoudiniSDR host by following that repository's host README
first. That install normally lands in a Python virtual environment prefix.
Activate it so `$VIRTUAL_ENV` is set, then verify:

```sh
source <your-houdini-venv>/bin/activate
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib
SoapySDRUtil --info            # prints versions and module paths
ls $SOAPY_SDR_PLUGIN_PATH      # must contain a Houdini .so module
```

If `SoapySDRUtil` is missing, or the module directory has no Houdini entry,
stop here and complete the SoapyHoudiniSDR host install first. Nothing in this
walkthrough can work without it.

One more thing comes from that repository: the framer teardown in section 8.4
imports `houdini_setup` from its host examples directory. The default location
is `~/repos/SoapyHoudiniSDR/host/examples`. If you keep it somewhere else,
export the path once:

```sh
export HOUDINI_EXAMPLES=<path-to-SoapyHoudiniSDR>/host/examples
```

This is the same environment variable the HIL tests under `tests/hil/` use for
the same dependency.

### 2.5 Get and build the sounder

Clone the repository and initialize the muFFT submodule. The build links muFFT
static libraries from the source tree, so skipping this step fails at link
time:

```sh
git clone <repo-url> <path-to-HoudiniLab>
cd <path-to-HoudiniLab>
git submodule update --init --recursive
```

Build muFFT, then the sounder:

```sh
cd <path-to-HoudiniLab>/CC/Sounder/mufft
cmake -B . -DCMAKE_BUILD_TYPE=Release && make

cd <path-to-HoudiniLab>/CC/Sounder
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

You should end up with `build/sounder`. The GPU beacon correlator is a separate
option, off by default, and the demo does not need it. Leave
`HOUDINI_USE_CUDA` alone unless you are specifically testing that path.

### 2.6 Point the demo at your bench

Edit `files/topology-houdini.json` and replace the two addresses with your own.
The base station goes under `BaseStations`, the client under `Clients`:

```json
{
    "BaseStations": { "BS0": { "sdr": [ "<bs-ip>" ] } },
    "Clients":      { "sdr": [ "<ue-ip>" ] }
}
```

Then open the config you plan to run (section 3) and check these fields against
your bench:

- `frequency` and `nco_frequency`: both default to 500 MHz. They must match
  each other for the matched NCO loopback path to work.
- `channel` and `ue_channel`: which RF channel each board uses.
- `ue_rx_gain_a` / `ue_tx_gain_a` and the `_b` pair: **these do nothing today,
  so do not spend bench time sweeping them.** The gain stages they name (LNA,
  PGA, TIA, PAD) belong to the Iris and USRP front ends that share this config
  format; the Houdini radio setup returns before those calls and discards the
  values.

  This is a "not wired up yet", not a "no such thing". The board does have gain
  control of its own: a digital step attenuator on the receive path, and QMC
  offset and gain in the RF data converter. Neither is exposed through the
  driver at the time of writing (the SoapyHoudiniSDR device README lists QMC
  among its deferred RFDC features), so there is no knob to turn from here yet.
  Expect that to change, and re-check this section against the driver's
  advertised gains before concluding the fields are still inert.
- `ue_power_ramp` and the `ue_ramp_*` gains are equally inert here: that block
  only runs under the Iris hardware framer, which these configs do not use.
- `ue_tx_advance_ticks`: the fine calibration that seats the client pilot
  inside the base station receive window. Start at 0 and sweep it if the pilot
  does not land (section 8).

Until a gain surface lands, signal level is set two other ways:

- **Physically**, by cabling and attenuation between the two boards. A direct
  cable normally needs an attenuator; an over the air path normally does not.
- **Digitally**, by the optional `tx_scale` field. Neither shipped config sets
  it, which means it is computed automatically to normalize the OFDM peak, and
  that is the right starting point. Set it explicitly only when you need to back
  the transmitted amplitude off.

Judge the level from the receive side rather than guessing at it. The
dashboard's magnitude panel shows the peak in dB, and running the sounder with
`HOUDINI_CL_RX_DEBUG=1` makes it print the received RMS and absolute maximum
periodically. Samples are 16 bit, so an absolute maximum near 32767 means you
are clipping and should attenuate; an RMS in the low tens means you are close
to the noise floor and the beacon correlation will be marginal.

Leave the frame geometry alone unless you know why you are changing it. The
shipped numbers are load bearing: 30 slots of 4096 samples is 122880 samples,
which at 122.88 MSPS is exactly 1 ms per frame, and `samps_per_slot` must stay
at or below 4096 to fit the FPGA transmit RAM.

## 3. Choose a config

Two configs ship with the demo. Both run one client at 122.88 MSPS.

| Config | Frame schedule | What you get |
|---|---|---|
| `files/houdini-1u.json` | `BGP` then guard | Channel estimate panels only |
| `files/houdini-ul.json` | Beacon at slot 0, pilot at slot 16, uplink data at slot 18 | Channel estimate **plus** the equalized constellation |

Start with `houdini-1u.json` to confirm the link is alive. Move to
`houdini-ul.json` once you see a clean channel estimate, because the
constellation panel only has something to draw when the frame carries an
uplink data slot.

In the schedule strings, `B` is the beacon, `P` is the pilot, `U` is uplink
data, and `G` is a guard slot. The uplink config places the pilot and data at
slots 16 and 18 rather than early in the frame, which keeps them clear of
beacon leakage at the base station.

## 4. Run the demo

You have two ways to start it. Mode A is one command and is the normal choice.
Mode B keeps the sounder in its own terminal, which is better when you are
debugging.

### 4.1 Mode A: the dashboard launches the sounder

```sh
cd <path-to-HoudiniLab>/CC/Sounder
python3 csi_gui/csi_server.py --launch --conf files/houdini-1u.json
```

The backend sets the environment, starts `sounder --view`, and retries the cold
start up to four times, which matters because radio discovery often fails on
the first attempt.

Before each attempt it also runs `csi_gui/teardown_framer.py` to release a
framer that a previous run may have left armed (section 8.4). You will see its
output prefixed `[teardown]`, and the sounder's prefixed `[sounder]`.

Two defaults assume one particular layout. If yours differs, override them:

- `--sounder-dir <path-to-HoudiniLab>/CC/Sounder` if the repository is not at
  `~/repos/HoudiniLab`.
- `--venv <your-houdini-venv>` if the SoapySDR virtual environment is not at
  `~/houdini_test`.

### 4.2 Mode B: run the two pieces yourself

Terminal 1, the dashboard backend:

```sh
cd <path-to-HoudiniLab>/CC/Sounder
python3 csi_gui/csi_server.py
```

Terminal 2, the sounder:

```sh
cd <path-to-HoudiniLab>/CC/Sounder
source <your-houdini-venv>/bin/activate
export LD_LIBRARY_PATH=$VIRTUAL_ENV/lib
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
export HOUDINI_MAX_FRAME=2000000000        # keep running instead of stopping at max_frame
./build/sounder --view --conf_file files/houdini-1u.json
```

Note the flag names differ between the two programs. The sounder takes
`--conf_file`; the dashboard backend takes `--conf`.

You do **not** need to start a separate beacon transmitter. With
`bs_hw_framer` set to true, the sounder arms the base station beacon itself.
The `beacon_tx_gold` binary in `tests/comms-func/` is a standalone test helper
from before that path existed, and it is not part of this demo.

### 4.3 Useful backend options

| Option | Default | What it does |
|---|---|---|
| `--http-port` | 8080 | Web server port |
| `--udp-port` | 9999 | Port the CSI datagrams arrive on |
| `--fps` | 30 | How often the page is pushed new data |
| `--stale-ms` | 1500 | Dim an antenna's plots when its last update is older than this (section 5.1) |
| `--mag-top` | 0 | Top of the fixed magnitude axis, in dB |
| `--mag-span` | 60 | Height of the fixed magnitude axis, in dB below `--mag-top` |
| `--csi-fps` | sounder default (30) | Per antenna stream rate out of the sounder |
| `--dest-host` | 127.0.0.1 | Where the sounder sends datagrams, when using `--launch` |

Run the backend on the same host as the sounder unless you have a reason not
to. If you split them, set `--dest-host` to the backend's address and make sure
UDP port 9999 is open between the two.

## 5. View the dashboard

The backend prints the URL as soon as it is listening:

```
[csi] dashboard at http://localhost:8080/  (SSH: -L 8080:localhost:8080)
```

If `<host>` is a remote machine, forward the port from your workstation:

```sh
ssh -L 8080:localhost:8080 <host>
```

Then open `http://localhost:8080/` in a browser. The page connects on its own
and reconnects if the stream drops, so you can leave it open across sounder
restarts.

The page serves three routes: `/` is the dashboard itself, `/stream` is the
Server Sent Events feed it reads, and `/vendor/tabler.min.css` is the
stylesheet. You will not normally open the last two by hand.

The button at the top right switches between the dark and light theme. Your
choice is stored in the browser, so it survives a reload and a restart of the
backend. It is the same control, and the same two themes, as the RayNet
compiler dashboard.

Each receive antenna gets one card with four panels:

1. **Magnitude of H in dB** across subcarriers. This is the frequency response
   of the channel. Nulls are real multipath fades, not faults.
2. **Phase in radians** across subcarriers. On a good capture this is a smooth
   ramp or a flat line. A ramp that changes slope frame to frame means the
   timing is drifting.
3. **Waterfall of magnitude**, time running downward. This is the panel that
   shows stability: a steady link draws smooth vertical streaks, and a link
   that keeps re-locking draws horizontal tearing.
4. **Constellation**, equalized uplink data. Only populated when you run
   `houdini-ul.json`. Clean QPSK shows four tight clusters.

Under the magnitude panel, sharing its subcarrier axis, is a fifth trace:

5. **Repeat quality**, the per subcarrier agreement between the pilot symbols
   inside one slot. The sounder already averages several repetitions of the same
   pilot to build H. This trace is how much they agreed. 1.0 means every
   repetition produced the same estimate, so H at that subcarrier is signal.
   A low value means they disagreed, so H there is noise and both the magnitude
   and the phase above it are meaningless at that subcarrier.

   Read it against the dashed line, not against zero. Pure noise does not
   average to 0, it averages to 1 divided by the number of repetitions, and the
   panel draws that floor for you. With six pilot symbols the floor is 0.17, so a
   trace sitting at 0.2 has no signal in it at all despite sounding respectable.
   The panel title states the repetition count and the floor.

   The panel needs at least two repetitions to say anything. With one it reports
   `not measurable` rather than drawing a trace, because the agreement of a
   single measurement with itself is always perfect. The status line carries the
   median across subcarriers as one number to watch.

### 5.1 When a card dims

If an antenna stops producing updates, its plots dim and a `stale 2.3 s` badge
appears next to the antenna name, counting up until fresh data arrives. The
card then returns to normal on its own.

This matters because the sounder refuses slots whose samples carry receive gaps
(section 8.3), so a losing link stops sending rather than sending something
untrue. Without the badge, the panels would simply hold their last good values,
and on a stationary bench a frozen display and a healthy static channel look
exactly the same. A dim card means "this is the last thing we knew, not what is
happening now".

The threshold is 1.5 seconds by default. If you lower `--csi-fps`, raise it to
match with `--stale-ms`, or every card will read as stale.

### 5.2 The ADC tab

Each card has two tabs. **Channel** is everything above. **ADC** shows the raw
converter samples for that antenna, which is where you look when the channel
panels are strange and you suspect the front end rather than the algorithm.

The trace is a minimum and maximum envelope of the whole slot, one band per
component, blue for I and orange for Q. It is an envelope rather than a
decimated copy of the samples on purpose: decimation keeps every Nth sample, so
a slot that clips for a handful of samples can produce a trace that never
touches the rail. With an envelope, one clipped sample pins its column to the
rail and cannot be hidden.

The dashed lines at the top and bottom are full scale. The status line under the
card reports the peak sample, that peak as a percentage of full scale, and the
exact count of samples at or above 99 percent of full scale, counted over every
sample in the slot rather than over the plotted envelope. A `clipping` badge
appears next to the panel title when that count is not zero.

Aim for a peak somewhere around half to three quarters of full scale. Much lower
wastes converter bits and the constellation gets noisy. At the rail the samples
are simply wrong and every panel downstream inherits it.

One caveat to know before you trust a reading of zero clipped samples. Full
scale here means the rail of the 16 bit sample format the sounder uses
throughout. If the converter ever delivers a narrower sample that is not shifted
up into the top of those 16 bits, the true rail is lower, the clipped count
stays at zero, and the peak reads as though there were headroom. The tell is the
peak itself: a peak that reports the same value on every frame is the rail,
whatever number it shows.

The tabs are per card, so you can watch one antenna's converter while another
shows its channel. That is how you find the single antenna that is clipping.

Every panel carries its tick labels in the margin beside the plot rather than
inside it, and both top panels use a fixed axis that never re-ranges. That is
deliberate. An axis that rescales itself every frame makes a static channel
look alive and hides slow drift, so the numbers next to a panel mean the same
thing in every frame you compare.

The magnitude axis runs from `--mag-top` down by `--mag-span` decibels. If the
trace leaves that window it would vanish off the top or the bottom of an
otherwise innocent looking empty panel, so an `off scale` badge appears next to
the panel title instead. If you see it, move the axis with `--mag-top` rather
than assuming the channel died.

Guard band and DC null subcarriers are drawn as gaps, not as zeros, so the
empty channel edges are expected.

## 6. Confirming it is actually working

The sounder prints one line when viewing mode initializes:

```
CSI view mode: streaming to 127.0.0.1:9999 (64 subcarriers, ~30 fps/ant, rx_conj=1, sym_start=120, timing_fix=1)
```

Check each field:

- The destination matches where your backend is listening.
- `rx_conj=1` on Houdini hardware. The receive mixer delivers baseband
  conjugated, and this flag undoes it. If it were wrong, every channel estimate
  would land on the mirror subcarrier and the constellation would scramble.
- `sym_start=120` for the shipped configs, which is the 128 sample prefix minus
  half the 16 sample cyclic prefix. Section 7 explains why.
- `timing_fix=1`, on by default for Houdini.

The backend prints a count every five seconds:

```
[csi] 1830 datagrams, antennas=[0]
```

If that count climbs steadily, the whole chain works. If it stays at zero while
the sounder is clearly running, the datagrams are not arriving: check that
`HOUDINI_CSI_UDP` points where the backend is bound, and that nothing between
the two is dropping UDP.

## 7. Tuning knobs

All of these are environment variables read by the sounder. Set them in the
same shell that launches it.

| Variable | Default | What it does |
|---|---|---|
| `HOUDINI_CSI_SYM_START` | `prefix` minus half the cyclic prefix (120 here) | Where the FFT window starts inside a received slot. An integer, or `auto` for the energy edge detector. |
| `HOUDINI_CSI_NO_TIMING_FIX` | unset | Set it to disable the per frame pilot re-alignment. |
| `HOUDINI_CSI_FPS` | 30 | Per antenna datagram rate out of the sounder. |
| `HOUDINI_MAX_FRAME` | from config `max_frame` | Frame count to run. Set large for continuous viewing. |
| `HOUDINI_CSI_UDP` | `127.0.0.1:9999` with `--view` | Where datagrams go, as `host:port`. |
| `HOUDINI_CSI_DUMP` | unset | One shot raw slot and H dump for offline analysis. |

`HOUDINI_CSI_SYM_START` is the one worth understanding. The cyclic prefix guard
is one sided. A window placed early, still inside the prefix, is a valid
circular shift and produces a pure phase ramp that the timing fix recovers. A
window even one sample late pulls the next symbol into the FFT and produces
inter symbol interference that no correction recovers. The old default sat
exactly on that cliff edge, so beacon re-lock jitter tipped runs into
interference at random. Backing the window off by half the cyclic prefix
centers it in the guard and gives margin on both sides. Measured on a window
interference run, this moved blind error vector magnitude from 19.8 percent to
3.2 percent.

The energy edge auto detector is opt in for a reason. Its 15 percent threshold
can trigger on pre symbol leakage and misalign the windows, so prefer a fixed
integer once you know the right value for your bench.

## 8. If something goes wrong

### 8.1 The dashboard shows "connecting" and never populates

The page is reaching the backend but no datagrams have arrived. Confirm the
sounder printed its `CSI view mode` line, then confirm the backend datagram
count is climbing (section 6). If the sounder never printed that line, it is
not in viewing mode: check that you passed `--view`, or that `HOUDINI_CSI_UDP`
is set.

### 8.2 Panels appear but the waterfall tears horizontally

The client is losing and re-acquiring the beacon. Usual causes, most likely
first: the two boards are not on a common reference clock (section 2.1), the RF
level is wrong so the beacon correlation is marginal, or `corr_scale` needs
adjusting for your path.

### 8.3 Channel estimate looks fine but the constellation is a smear

Confirm you are running `houdini-ul.json`, since the other config has no
uplink data slot to equalize. If you are, work through the three causes below
in order.

**Cause 1: the FFT window is sitting late and taking in interference.** Try
`HOUDINI_CSI_SYM_START` a few samples lower and watch the constellation
tighten. Section 7 explains the asymmetry: early is recoverable, late is not.

**Cause 2: dropped receive packets.** When a receive packet is lost,
`Radio::recvHoudini` zero-pads the hole so the rest of the window keeps its
true timing. Those zeros are not signal, and an FFT taken across them produces
a wrong channel estimate. Because the estimate is cached per antenna and reused
to equalize every following uplink data slot, accepting one damaged pilot used
to smear frames until the next clean pilot replaced it, so a single lost packet
showed up as a burst of smearing plus a phase jump rather than one bad frame.

Viewing mode now refuses those slots instead of rendering them. When a slot
arrives carrying padded samples it is dropped, that antenna's card dims with a
`stale` badge (section 5.1), and the sounder logs, at most once every five
seconds:

```
CSI view: dropped 12 slot(s) with RX gaps (latest 848 padded samples, ant 0)
```

So the display holds its last good estimate rather than showing a false one,
and marks it as old rather than passing it off as current. **A dimmed card plus
that warning means the link is losing packets**, and the fix is on the link, not
in the viewer. Recording mode is unchanged: it still
keeps every sample and records the damaged ranges in the file's gap table.

If you saw this before the fix landed, note that a clean recording was never
evidence of a clean link. The recording is not cleaner; it just carries the gap
table that makes the damage findable. To confirm the rate, record a capture
over the same link and compare its gap table against how often the warning
appears. Tracked as AP-10.

**Cause 3: a failed uplink data transmission at the client.** The constellation
is built from the client's uplink data slot. Watch the sounder log for
`BAD Write`, `BAD Write, UL data`, and `unexpected writeStream error`. The
uplink data burst now checks its own transmit return, so a short write on that
slot reports itself rather than appearing only as a smear.

One blind spot remains: no code polls the driver's asynchronous transmit
status, so a burst that was accepted but sent late, or dropped for being late,
is still never reported. A late pilot on the fine timing grid produces exactly
the phase jumps you would be chasing, so treat a smear with no logged warning
as still consistent with this cause. Tracked as AP-10.

### 8.4 The sounder will not start, and discovery looks broken

A run that was killed rather than stopped can leave the base station framer
armed and the transmit RAM loaded. The next run then fails to start, and it
usually looks like a discovery or network problem rather than leftover state.

Clear it:

```sh
cd <path-to-HoudiniLab>/CC/Sounder
python3 csi_gui/teardown_framer.py
```

It reads the radio addresses from `files/topology-houdini.json`, opens each
one, issues the framer abort, clears the transmit RAM, and releases the gate.
Point it elsewhere with `--topology <file>`, or name radios directly with
`--node <addr>` (repeatable).

Read the exit status, not just the output. It is 0 only when every radio was
cleared, and non-zero when one could not be opened or torn down, which is
normally the actual reason the sounder then fails. `--launch` runs this for you
before each attempt and prefixes its output `[teardown]`.

This script opens a connection to each radio, so it is a device-touching
operation. Do not run it against boards someone else is using.

If it cannot import `houdini_setup`, set `HOUDINI_EXAMPLES` (section 2.4).

If the teardown itself reports that an RX stream is still open, a dead process
left one behind and no teardown can close another process's stream. Restart the
server on that node and try again:

```sh
ssh <user>@<radio-ip> 'sudo systemctl restart SoapySDRServer'
```

That is also the recovery when a radio open fails with
`SoapyRPCUnpacker::recv() TIMEOUT`.

### 8.5 A radio is discoverable but still will not open

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

### 8.6 Client never finds the beacon

Check in this order: both boards on a common clock, the RF path is actually
connected and at a sane level, `frequency` and `nco_frequency` match each other
in the config, and the beacon board is really the one named under
`BaseStations` in the topology file.

## 9. Clean up

Press Ctrl+C in the backend terminal. In mode A it kills the whole sounder
process group on the way out, so one Ctrl+C stops everything. In mode B stop
the sounder in its own terminal as well.

Stop the whole process group, not just the backend. `--launch` starts the
sounder in a child shell that keeps running if you kill only `csi_server`; the
orphan then starts a SECOND sounder against the same radios, both streaming to
the same UDP port. The dashboard interleaves frames from both and the display
cannot be trusted in either direction. If you started it detached, kill by
group (`kill -- -<pgid>`), then confirm:

```sh
pgrep -cx sounder      # want 0 when stopped, exactly 1 while running
```

After any run that ended abnormally, release the framer before starting again:

```sh
python3 csi_gui/teardown_framer.py
```

A framer left armed is the most common reason the next run fails to start, and
the failure does not look like leftover state (section 8.4).
