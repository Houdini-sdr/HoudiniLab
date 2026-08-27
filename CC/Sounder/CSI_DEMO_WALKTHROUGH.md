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
  HTML5 canvas, with no external JavaScript libraries, so it works on a host
  with no internet access.
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
- `ue_rx_gain_a` / `ue_tx_gain_a` and the `_b` pair: set for your RF path. A
  cabled loopback with an attenuator needs very different gains from an over
  the air link.
- `ue_tx_advance_ticks`: the fine calibration that seats the client pilot
  inside the base station receive window. Start at 0 and sweep it if the pilot
  does not land (section 8).

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

Two cautions about this path:

1. The launcher hardcodes the virtual environment as `~/houdini_test` and the
   source directory as `~/repos/HoudiniLab/CC/Sounder`. If yours differ, pass
   `--sounder-dir <path-to-HoudiniLab>/CC/Sounder` and use mode B, or edit
   `_launch_sounder` in `csi_gui/csi_server.py`.
2. The launcher calls `/tmp/td.py` before each attempt to tear down a stuck
   framer. That script is not in this repository. See section 8.4.

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

The page serves two routes: `/` is the dashboard itself, and `/stream` is the
Server Sent Events feed it reads. You will not normally open `/stream` by hand.

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
uplink data slot to equalize. If you are, this is usually the FFT window
sitting late and taking in interference. Try `HOUDINI_CSI_SYM_START` a few
samples lower and watch the constellation tighten. Section 7 explains the
asymmetry: early is recoverable, late is not.

### 8.4 The teardown script `/tmp/td.py` is missing

Known gap. The `--launch` path runs `timeout 60 python3 /tmp/td.py` before each
attempt, to release a framer left armed by a previous run. That script is not
in this repository and not shipped anywhere. If it is absent the command fails
silently and the launcher continues, so `--launch` still works, but a genuinely
stuck framer will not be cleared and the sounder will keep failing to start.

Until it is vendored, tear down by hand using the helper this repository does
ship. From `tests/hil/`, `beacon_tdd._teardown(sdr)` issues an abort, clears
the transmit RAM, and releases the gate. Alternatively, restart the Houdini
server on the affected board.

### 8.5 Client never finds the beacon

Check in this order: both boards on a common clock, the RF path is actually
connected and at a sane level, `frequency` and `nco_frequency` match each other
in the config, and the beacon board is really the one named under
`BaseStations` in the topology file.

## 9. Clean up

Press Ctrl+C in the backend terminal. In mode A it kills the whole sounder
process group on the way out, so one Ctrl+C stops everything. In mode B stop
the sounder in its own terminal as well.

After any run that ended abnormally, confirm the base station framer is not
left armed before starting again (section 8.4). A framer left armed is the most
common reason the next run fails to start.
