# Running the live CSI dashboard demo, step by step

This walkthrough goes from an empty machine to a live channel display in a web
browser: install the dependencies, build the sounder, point it at your own two
radios, run the base station and client together, and read the panels the
dashboard draws. Every step shows the exact command and what you should see.

If you are on the lab bench, `DEMO_BENCH_RUNBOOK.md` next to this file fills
in every placeholder below with the exact machines, paths, and commands the
live demo runs with. This walkthrough stays generic so it works on any bench.

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
- Two radios take part. The base station arms a hardware TDD schedule built
  from the config frame (one slot per schedule character), transmits a single
  496 sample beacon once per frame from its replay RAM, and runs one
  continuous receive covering every receive slot. The client hunts for the
  beacon, confirms it twice on the frame grid behind a sync SNR floor, anchors
  its own frame timing to it, and from then on transmits one zero padded burst
  per frame that seats the pilot and the uplink data in their scheduled slots
  to the sample. A periodic targeted re-sync checks the beacon is still where
  the anchor predicts; repeated failure escalates to a full re-acquisition,
  and both events are logged. The design and its verification live in
  `DEMO_VERIFICATION.md`.
- The channel estimate is pilot agnostic. It correlates against the frequency
  domain reference built from your config, so an LTS, a Zadoff Chu, or any
  other `pilot_seq` works without code changes.
- The dashboard draws one card per receive antenna and scales automatically to
  however many antennas appear in the stream. Above them sits a single **beacon
  sync** card per client, which describes the LINK rather than any one antenna
  (section 5.2).

## 2. Install everything (from nothing)

### 2.1 What you need before starting

- A compute host `<host>`: a Linux machine (x86_64 or aarch64) with network
  reach to both radios.
- Two Houdini radio nodes at `<bs-ip>` and `<ue-ip>`: powered, running the
  Houdini server, on a firmware stack your team has blessed. Board bring up is
  owned by the SoapyHoudiniSDR and Houdini-Streaming projects and is not
  covered here. If you did not set the boards up yourself, ask whoever did.
- **Both boards driven from one reference clock.** This is not optional. Two
  free running boards drift far enough that the client pilot walks across the
  whole base station frame and never stays inside the receive window. Feed both
  boards a common 10 MHz on `CLK IN`
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

Pilot and data placement is sample exact: the client pads each burst so its
start escapes the driver's 3125 ns scheduling grid, and `tx_advance` in the
config is a bench calibration that seats the burst at its nominal in-slot
position. If you change cabling or the RF path, re-derive it with the
procedure in the config's `_tx_advance_note`.

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
  `~/repos/HoudiniLab`. Check this one carefully on a host with more than one
  checkout: the launcher runs whatever `build/sounder` it finds under this
  directory, and a stale binary from another checkout looks exactly like the
  current demo until a log line you expect is missing. When in doubt, verify
  with `strings <dir>/build/sounder | grep <a-string-only-the-new-code-logs>`.
- `--venv <your-houdini-venv>` if the SoapySDR virtual environment is not at
  `~/houdini_test`.

That command on its own is deliberately quiet. It prints the teardown, the
sounder's startup and a `[csi]` datagram counter roughly once a second, and
almost nothing per frame. That is the normal amount of output, so do not read
a short log as a sign that something is wrong.

If you want the per frame diagnostics instead, export them in the same shell
before launching. Section 7 describes each one:

```sh
export HOUDINI_BS_RX_DEBUG=1 HOUDINI_UE_TX_DEBUG=1 HOUDINI_CSI_R_DEBUG=1
export HOUDINI_CFO_LOG_EVERY=1
python3 csi_gui/csi_server.py --launch --conf files/houdini-1u.json
```

On a healthy cabled bench this is a large difference in output and no
difference in behaviour. Two back to back runs on the same bench measured 6,189
lines in 60 seconds with the exports set against 342 lines in 85 seconds
without them, while the `[csi]` datagram counter advanced by an identical 443
per reporting interval in both. So a quiet log means the exports are unset, not
that the demo is running slowly. Check the datagram counter, not the line rate.

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
| `--mag-top` | 90 | Top of the fixed magnitude axis, in dB |
| `--mag-span` | 40 | Height of the fixed magnitude axis, in dB below `--mag-top` |
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

The cards fill the width of the window and reflow as you resize it, so a wider
window gives you more cards side by side rather than more empty space. The panels
inside a card stretch with it, so making the window wider makes every plot bigger.

Each receive antenna gets one card with four panels:

1. **Magnitude of H in dB** across subcarriers. This is the frequency response
   of the channel. Nulls are real multipath fades, not faults.
2. **Phase in radians**, as two stacked panels. The **raw** panel shows the
   phase exactly as measured: the deliberate FFT window back-off inside the
   cyclic prefix rides a steep linear ramp on it, which wraps every few
   subcarriers and draws a sawtooth. The tooth spacing is a delay gauge: with
   the shipped 8 sample back-off, expect a tooth every 8 subcarriers, plus or
   minus the run's small extraction draw. The **corrected** panel removes
   that known instrumental ramp and anchors the run's arbitrary common phase
   to zero at the third displayed update (the first two settle), so what remains is physical: its tilt is the
   run's residual timing (a fraction of a sample to a sample or two), and any
   movement after the anchor lands is a real event. The common phase re-draws
   every restart because the two nodes are frequency locked, not phase
   locked; only the corrected panel hides that lottery, on purpose.
3. **Waterfall of magnitude**, time running downward. This is the panel that
   shows stability: a steady link draws smooth vertical streaks, and a link
   that keeps re-locking draws horizontal tearing.
4. **Constellation**, equalized uplink data. Only populated when you run
   `houdini-ul.json`. Clean QPSK shows four tight clusters.

Guard band and DC null subcarriers are drawn as gaps in every per-subcarrier
panel, never as zeros: nothing is transmitted there, so nothing is measured
there. Expect gaps at the band edges and one at DC.

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

`--stale-ms` sets the refresh floor for BOTH card types. The sync card decides
what to display using its own thresholds, but its age can only advance when the
server pushes an event, and that decision uses `--stale-ms`. So raising it
delays how quickly the sync card notices a quiet link too (section 5.2).

### 5.2 The beacon sync card

One card per client, above the antenna cards. It answers a different question
from the channel panels: not "what does the channel look like" but "is the UE
still locked to the base station's beacon, and how far off is it".

The trace is `resid`: how many samples the detected beacon landed from where the
anchored grid predicted it. The shaded band is the acceptance tolerance the
sounder actually applies, carried on the wire rather than hardcoded in the page,
so it always matches the running code. Healthy looks like a flat line on zero
well inside the band.

The x axis is the FRAME NUMBER, not the point index. Detections are irregularly
spaced, and a wide gap is itself information, so an evenly spaced axis would
hide it.

The badge reads one of:

| Badge | Meaning |
|---|---|
| `LOCKED` | Beacon found where the anchored grid predicted it. Normal. The UE still nudges its schedule on one of these, by a fraction of the measured residual, so a locked run is a tracking run and not a frozen one. |
| `HOLD PENDING` | One off-grid detection seen. Deliberately NOT acted on: single large offsets are scatter, so the sounder waits for a second consistent one. |
| `RE-ANCHORED` | The UE gave up tracking and re-acquired. The readout names the schedule step applied. This is the LARGE move; the small per detection nudge on a `LOCKED` record is the other one. |
| `WEAK BEACON` | Something was detected but it failed the SNR floor. Different from no beacon at all, and usually means levels or cabling. |
| `RE-ANCHOR FAILED` | An escalation ran and re-acquisition did not confirm. The previous anchor is being kept. |
| `NOT SYNCED` | No detections at all, or the stream has been silent for a minute. |

A badge may be suffixed `quiet 4.2s`. That is NOT a fault. The UE reports only
when it makes a detection, and it only attempts one when the anchored grid
predicts the beacon inside the read window, so seconds of silence between bursts
are normal on a perfectly healthy link. The plot dims while quiet so you can see
at a glance that you are looking at held data rather than live data.

How long is normal changed on 2026-09-02. The client now looks at the beacon
every 2.6 seconds by default rather than every 260 milliseconds, because the
measured clock stability supports coasting far longer than the old cadence
assumed. So a healthy link is quiet most of the time, and the badge only means
something if it is much longer than the cadence. The page works this out for
itself: it measures the interval between the reports it actually receives and
marks the card quiet at three times that, so the badge keeps its meaning if the
cadence is changed again. Nothing to configure.

The readout line carries three figures in ppm, and they are not three views of
one number. Read them in this order.

1. **clock** is the total offset the client's timing tracker currently holds. It
   comes from the frame period the tracker has learned, and on a two board bench
   running on internal references it reads several ppm. It is a filtered state
   rather than a set of samples, so it is printed as a value with no error bar.
2. **residual** is what is left after the tracker has done its work, fitted from
   the slope of the resid trace. It should sit near zero. This is the number that
   tells you the loop is closed.
3. **beacon** is a separate instrument. It measures the same total offset from
   the beacon's own carrier phase, independently of the timing channel, and it
   carries a standard error because each detection is an independent reading.

The cross check that means something is **clock against beacon**, because those
two measure the same quantity two different ways. Do not compare clock against
residual: residual is clock's own error term, so the two are meant to differ by
whatever factor the tracker is winning by, and a large ratio there is the loop
working rather than a fault.

Expect the beacon figure to sit off the clock figure by a fixed amount on any
given bench. The beacon estimator is precise but carries a configuration
dependent bias, so treat it as a liveness and sanity instrument rather than the
value to correct with. It is annotated when it falls inside the phase noise
floor: a short correlation lag turns a tiny phase error into an apparently large
frequency, so treat sub kilohertz beacon readings as instrument noise rather
than a real offset. It reads `beacon n/a` when the visible segment holds no
detection to estimate from.

### 5.3 The ADC tab

Each card has two tabs. **Channel** is everything above. **ADC** shows the
received pilot slot in the time domain, which is where you look when the
channel panels are strange and you suspect the front end or the timing
rather than the algorithm.

The trace is the slot's **power envelope in dBFS** on a fixed 0 to -80 dB
axis: each plotted column carries the maximum absolute sample over every
sample it covers, so a single clipped sample pins its column at 0 dBFS and
cannot be hidden. Two dashed vertical markers show the nominal guard seats,
`ofdm_tx_zero_prefix` and `ofdm_tx_zero_postfix` samples in from the slot
edges (read from your config): on a healthy run the burst's
rising edge sits on the first marker and its falling edge on the second, so
this panel doubles as a live landing view for the transmit timing.

The absolute question, how much of the converter you are using, is answered
underneath by a bar on a fixed full scale. The bar turns amber below 10
percent, meaning under driven, and red at 95 percent or on any clipped
sample. The status line reports the pilot peak in counts and as a percent of
full scale, plus the peak and clipped count across every slot of the frame,
not just the plotted one. A `clipping` badge appears next to the title when
the clipped count is not zero.

Aim for a peak somewhere around half to three quarters of full scale. Much
lower wastes converter bits and the constellation gets noisy. At the rail
the samples are simply wrong and every panel downstream inherits it.

One caveat to know before you trust a reading of zero clipped samples. Full
scale here means the rail of the 16 bit sample format the sounder uses
throughout. If the converter ever delivers a narrower sample that is not
shifted up into the top of those 16 bits, the true rail is lower, the
clipped count stays at zero, and the peak reads as though there were
headroom. The tell is the peak itself: a peak that reports the same value on
every frame is the rail, whatever number it shows.

The tabs are per card, so you can watch one antenna's converter while
another shows its channel. That is how you find the single antenna that is
clipping.

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

Other sounder lines worth recognizing on a healthy run:

```
UE pilot burst: scheduled 97 frames up to <tick> (pad 148)
Re-sync frame 1255: beacon alive on the anchored grid (resid +0 within scatter, snr 47.6 dB), tid 0
```

The first appears with `HOUDINI_UE_TX_DEBUG=1` and shows the client keeping
its transmit queue topped up. The second appears on every targeted re-sync
attempt; `resid` near zero and an SNR in the mid 40s dB on a cabled bench mean
the anchor is holding. If the client loses the link, the base station side
prints `BS: UE PILOT LOST for N consecutive frames` and, when it returns,
`BS: UE pilot RETURNED`.

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
| `HOUDINI_SYNC_SNR_DB` | 30 | Sync SNR floor in dB. Detections below it are rejected during acquisition and re-sync. The metric reads true link SNR; a cabled bench measures the mid 40s. |
| `HOUDINI_PILOT_HORIZON` | from config `ue_pilot_horizon` (96) | How many frames of client bursts are queued ahead of real time. Larger survives slower host loops; every extra frame delays a timing correction reaching the wire. |
| `HOUDINI_BS_RX_DEBUG` | unset | Base station prints its rederivation of the client schedule (`pilot_grid_off`, `pu_spacing_err`). Both should sit within one sample of zero. |
| `HOUDINI_UE_TX_DEBUG` | unset | Client prints its burst scheduling (frames queued, pad). |
| `HOUDINI_CSI_R_DEBUG` | unset | Recorder prints the per frame pilot re-alignment it chose (`r`, and the blind score behind it), one line per 30 corrections. |
| `HOUDINI_CFO_LOG_EVERY` | 10 | How many beacon detections pass per `Beacon CFO` line. The default logs one in ten, so a quiet run is expected. Set it to 1 for a calibration run where you want every estimate. |
| `HOUDINI_CNS_DUMP_LOW` | unset | Directory for autopsy dumps of the first few low scoring constellations. The directory must already exist. |

### 7.0 Choosing the beacon waveform

The base station transmits a short burst at the top of every frame and the
client finds it by correlation. Which burst it sends is a config field,
`beacon_type`, in the `tdd_conf` block. Five are available:

| value | what it is |
| --- | --- |
| `legacy` | The default. Fifteen repeats of a 16 sample training symbol, then two repeats of a 128 sample Gold sequence. |
| `legacy_guard` | The same, with a 32 sample cyclic guard inserted before the Gold field, in the style of an 802.11 long training field. |
| `dot11` | The 802.11a/g/n legacy preamble as the standard defines it: the short training field, then the guard and two long training symbols. |
| `nr` | The 5G NR primary synchronisation signal, then a guard and two repeats of a tracking symbol built from the NR reference sequence. The client finds it on the repeated tracking symbol. |
| `nr_pss` | The same burst as `nr`, sample for sample, but the client finds it the way an NR handset does: a plain matched filter on the primary synchronisation signal, with no repeat check. The log says `threshold form forced to nolag` when this is in effect. |

The first four were measured on the bench, four rounds each with the order
rotated, about 8000 detections apiece, and all five have since been run end to
end through the client (`nr_pss` on 2026-09-03, `DEMO_VERIFICATION.md` 8z). The margin figures below were measured with the older
comparison rule, so read them as a ranking rather than as absolute numbers. **Timing is the same for all of them**, within
measurement error. What separates them is detection margin: the worst detection
of the run cleared the threshold by 12x for `legacy` and `legacy_guard`, 7x for
`dot11`, and only 2.3x for `nr`.

**Leave it at `legacy` unless you have a reason.** The margin is the best of the
four, and it is the waveform every other measurement in this repository was
taken against. If you set a value that is not in the table the client refuses to
start rather than falling back, so a typo cannot quietly leave you on a
different beacon than you think.

The one thing the alternatives are better at is the beacon's own frequency
estimate, where `dot11` is about a third more stable. That number is a
secondary reading on the sync panel, not what the client actually tracks
frequency with, so it does not currently justify the margin it costs.

### 7.1 Free running clock knobs

These belong to the timing tracker the client runs when the two boards are on
their own references rather than a shared one. Every default below is a measured
value, not a guess, and the run is expected to be correct with all of them
unset. They exist so a bench with different jitter or a different detection rate
can be swept without a rebuild. Change one at a time and record which one.

| Variable | Default | Meaning |
| --- | --- | --- |
| `HOUDINI_SCATTER_TOL_US` | 2.0 | How far a detection may land from the tracked grid and still count as the same beacon, in microseconds. Expressed in time so it holds at every sample rate. Raising it past about 15 makes the accept window larger than the slot can present, and the code clamps it and says so. Lowering it WIDENS the window the client searches in, so a tighter tolerance finds more beacons, not fewer. |
| `HOUDINI_CONFIRM_TOL_US` | 5.2083 | The same tolerance during acquisition, where two further detections must land on the first one's grid before the lock is trusted. Tighter than the tracking gate on purpose. |
| `HOUDINI_SYNC_TOL_SAMPLES` | a quarter of the config `ofdm_tx_zero_prefix` | Timing slack budgeted to drift between two beacon checks. Half the input to the resync cadence. |
| `HOUDINI_SYNC_RESIDUAL_PPM` | 0.1 | Assumed worst case clock error after tracking. The other half of the cadence input. The two together set how often the client looks at the beacon, which at these defaults is every 2.6 seconds. |
| `HOUDINI_GRID_ALPHA` | 0.5 | How much of each accepted detection's residual is applied to the schedule. Set both alpha and beta to 0 for a fixed period grid with no tracking at all. |
| `HOUDINI_GRID_BETA` | 0.1 | How much of that residual is applied to the frame period estimate. |
| `HOUDINI_GRID_STEP_PPM` | 0.5 | The most one detection may move the period estimate, in ppm. Guards against a single scatter outlier levering the rate. |
| `HOUDINI_GRID_MAX_PPM` | 100 | Absolute band the period estimate may occupy either side of nominal. A plausibility bound, not an outlier reject. |
| `HOUDINI_GRID_TRUST_PPM` | 1.0 | How far the tracked period and a fresh acquisition confirm may disagree before an escalation throws away the tracked one. |
| `HOUDINI_ACQ_REFINE_SPAN` | 200 | Frames of baseline the acquisition stage wants before it trusts its rate estimate. Longer is more accurate and slower to lock. |
| `HOUDINI_ACQ_MAX_PPM` | 100 | The same plausibility band applied to a rate that acquisition hands back. |
| `HOUDINI_CFO_INDEX_GUARD` | 8 | Samples of margin the carrier estimator requires around the beacon before it will run. Out of range, it reports nothing rather than a fabricated figure. |
| `HOUDINI_CSI_NO_PHASE_FIX` | unset | Set it to disable the per symbol common phase correction. On by default for Houdini. The residual carrier figure in the log is measured before this correction either way, so the health reading survives turning it off. |
| `HOUDINI_COALESCE_SLOTS` | 1 | Collapse runs of discarded slots into one read. On by default. Set it to 0 for a per slot A/B comparison. It changes the client loop rate by roughly 2x, so any figure quoted per iteration has to say which setting produced it. |
| `HOUDINI_LOOP_PROFILE` | 0 (off) | Print a breakdown of where a client loop iteration goes, once per this many iterations. Counts ITERATIONS. |
| `HOUDINI_RX_PROFILE` | 0 (off) | Print the split between draining the FIFO and the read itself, once per this many radio reads. Counts READS, which is a different scale from the line above, and coalescing changes the ratio between them. |
| `HOUDINI_CONFIRM_TOL_US` | 5.2083 | See the acquisition row above. Never applied looser than the tracking gate, whatever you set. |
| `HOUDINI_BEACON_PICK` | unset | Diagnostic. Unset means the search returns the strongest correlation peak and then looks back a short way for the earliest arriving path, which is the shipped behaviour. `argmax` skips the look-back and takes the strongest peak; `first` restores the pre-2026-09-02 rule of taking the earliest crossing. Neither older rule is safe. The earliest-crossing rule locks onto the beacon's own preamble a few hundred samples early once the link is strong enough, and whether it does depends on received level, so a link that is fine today fails after a gain change. The strongest-peak rule is fine on a cable but wrong over the air, where the strongest arrival is often a reflection and which one is strongest changes as the signal fades, making the timing reference jump. Leave it unset. |
| `HOUDINI_BEACON_THRESH` | unset | Diagnostic. Unset means the detector compares a normalised correlation, so the `corr_scale` setting means the same thing at every signal level. `power` restores the older comparison, whose effective strictness changed with received level by a factor of ten thousand across the range we measured. `nolag` uses a plain matched filter with no repeat check; it is worse on the repeated-field beacons and is here for comparison. The `nr_pss` beacon selects this form by itself, because its reference does not repeat. Leave it unset. |
| `HOUDINI_FIRST_PATH_WIN` | 64 | How far back, in samples, the search looks for an earlier arriving path. 64 samples is about half a microsecond, which covers indoor multipath with margin. A longer window suits an outdoor path, but only with a beacon whose preamble sits further away than the window, or the preamble comes back into range. |
| `HOUDINI_FIRST_PATH_DB` | -9 | How much weaker, in dB, an earlier arrival may be and still be taken as the first path. The default clears our hardest test case by under a decibel, so treat it as tuned rather than generous, and re-run the offline beacon test after changing it. |
| `HOUDINI_BEACON_FS` | 0.6 | Diagnostic. What fraction of full scale the base station transmits the beacon at. Lowering it weakens the beacon and nothing else, which makes it a stand-in for path loss on a cabled bench. Below about 0.05 the client stops acquiring. |
| `HOUDINI_RESYNC_RETRY_MAX` | 100 | Misses in one resync period before the client logs an exhausted episode. The anchored grid keeps flying either way; this is a counter, not a give-up. Note the resync period is now 2.6 seconds, so one episode covers far more wall time than it used to. |
| `HOUDINI_ESCALATE_EPISODES` | 2 | Consecutive exhausted episodes before the client abandons tracking and re-acquires from scratch. |
| `HOUDINI_HOLD_OFFGRID` | 2 | Consecutive off grid detections before the beacon counts as MOVED. One is scatter, so the default waits for a second. |
| `HOUDINI_BS_RX_EVERY` | 20 | How many base station frames pass per `HOUDINI_BS_RX:` line. Only has an effect when `HOUDINI_BS_RX_DEBUG` is set. Set it to 1 for a two way transfer measurement, which needs every frame; expect a very large log. |
| `HOUDINI_TRACKER` | `ab` | Which estimator tracks the base station frame grid. `ab` is the shipped alpha beta. `kf` selects the Kalman, which carries the gap length in its own uncertainty rather than applying fixed gains to every gap. Leave it unset unless you are running the comparison. |
| `HOUDINI_KF_INNOV_GATE` | 4.0 | Kalman only. How many sigma an observation may sit from the prediction before the filter refuses it. This is the Kalman's outlier defence and it is what makes that arm worth running; set it to 0 to disable and the arm gets measurably worse. |
| `HOUDINI_KF_MEAS_VAR` | 0.5 | Kalman only. Assumed detector scatter variance in samples squared. The default is the measured 0.63 to 0.70 sample spread, squared. |
| `HOUDINI_KF_RATE_RW` | 1e-9 | Kalman only. How fast the frame period is assumed to wander, in samples squared per frame cubed. The default comes from the 0.23 ppm the clock moved across one session. |

The last four are the escalation net. Their defaults were tuned when the client
grid drifted out of tolerance in about a second. With the clock steered it holds
for minutes, so the defaults are conservative by a wide margin and are expected
to be retuned. Change them one at a time against a known good baseline run, and
keep the net rather than removing it: it is what stands between a lost beacon
and a client that flies on stale timing without saying so.

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

### 8.0 The run produces almost no output, or says it cannot find a radio

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

The usual cause is how the previous run ended. The launcher starts the sounder
underneath a small retry loop, so if you kill the launcher, the retry loop keeps
going and starts another sounder, and that sounder holds both boards. Killing
the window you launched from is not enough.

Release it, from the rig, in the sounder directory:

```sh
python3 tools/rig_release_holders.py
python3 csi_gui/teardown_framer.py
```

The first command finds and stops any leftover sounder on the rig host. The
second confirms the boards are clear; it prints `all 2 radio(s) clear` when they
are. Then start your run again.

If the release tool finds nothing and the boards are still held, something
outside your session is holding them, and the board's own server has to be
restarted:

```sh
sudo systemctl restart SoapySDRServer
```

That needs a password, so on a shared bench it is worth asking whether a
colleague is using the boards before reaching for it.

### 8.1 The dashboard shows "connecting" and never populates

The page is reaching the backend but no datagrams have arrived. Confirm the
sounder printed its `CSI view mode` line, then confirm the backend datagram
count is climbing (section 6). If the sounder never printed that line, it is
not in viewing mode: check that you passed `--view`, or that `HOUDINI_CSI_UDP`
is set.

### 8.2 Panels appear but the waterfall tears horizontally

The client is losing and re-acquiring the beacon. Confirm from the log before
guessing: escalations print `re-sync escalation` with a reason, and the base
station prints `UE PILOT LOST` during the outage. Usual causes, most likely
first: the two boards are not on a common reference clock (section 2.1), the
RF level is wrong so detections fall under the sync SNR floor (the re-sync
lines print the measured SNR; compare it against `HOUDINI_SYNC_SNR_DB`), or
`corr_scale` needs adjusting for your path.

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

**Cause 3: a failed transmission at the client.** The pilot and the uplink
data ride one composed burst per frame, and its transmit return is checked:
watch the sounder log for `BAD Write` and `unexpected writeStream error`. The
client also drains the driver's asynchronous transmit status once per queue
top-up, so a burst that was accepted but later reported late or dropped is
surfaced rather than lost.

If the constellation is a ring or smears differently from one restart to the
next with none of the above logged, that class of fault was root caused and
fixed in the 2026-08-30 campaign (timing offsets between the pilot and data
paths; `DEMO_VERIFICATION.md` rows 4.36 to 4.48). On current code a persistent
smear points at RF level, clipping, or receive gaps, not at restart luck.

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

Acquisition requires more than a correlation peak: the detection must clear
the sync SNR floor (`HOUDINI_SYNC_SNR_DB`, default 30 dB of true link SNR) and
then repeat twice at exactly one frame spacing before the client anchors. A
marginal RF path can therefore correlate occasionally yet never acquire. The
re-sync and acquisition log lines print the measured SNR; on a cabled bench
expect the mid 40s dB, and treat much less as an RF level or cabling problem
rather than a software one.

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
