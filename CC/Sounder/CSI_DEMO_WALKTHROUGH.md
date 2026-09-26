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

Additional reference material lives in `../../docs/archive/UE_TX_FINE_GRID_TIMING.md`
(why the client pilot lands on the fine timing grid) and
`../../docs/archive/TWO_BOARD_CLOCK_LOCK.md` (locking both boards to one external
reference clock). The record of every measurement behind this walkthrough is
`DEMO_VERIFICATION.md`.

## 0. Quick start

If `<host>` already has the sounder built and the SoapyHoudiniSDR host plugin
installed (section 2 if not), these steps take you from a shell to a live
display. Every command runs on `<host>` unless it says otherwise.

1. Activate the plugin's environment and go to the sounder:

   ```sh
   source <your-houdini-venv>/bin/activate
   cd <path-to-HoudiniLab>/CC/Sounder
   ```

   For a slots config (the demo, section 3), also select the slots host
   plugin in the same shell before steps 3 and 4:

   ```sh
   export HOUDINI_SOAPY_ROOT=<slots-plugin-prefix>
   ```

2. Pick a config from section 3, then put your two radios' addresses in the
   topology file it names. This prints the file name:

   ```sh
   grep serial_file files/<config>.json
   ```

   Put the base station's address under `BaseStations` and the client's under
   `Clients` (section 2.6 shows the layout).

3. Check the setup. Every FAIL line says how to fix it; fix them and run it
   again until it prints `Ready.`:

   ```sh
   python3 csi_gui/check_setup.py --conf files/<config>.json
   ```

4. Start the dashboard with its controls:

   ```sh
   python3 csi_gui/csi_server.py --control --conf files/<config>.json
   ```

5. On your workstation, forward the port and open the page:

   ```sh
   ssh -L 8080:localhost:8080 <host>
   ```

   Then browse to `http://localhost:8080/`.

6. In the page header, choose the config in the list and press **Start**. The
   dashboard checks the setup again, clears the radios, and starts the
   sounder. If the check finds a problem it shows the list with the fix instead
   of starting.

7. What good looks like, usually within a minute:
   - the beacon sync card reads `LOCKED` (section 5.2);
   - one card per receive antenna appears, and its channel estimate updates;
   - with an uplink config, the constellation shows tight clusters and the
     quality line shows the MER.

   If the sync card stays at `NOT SYNCED`, go to section 8.6. Section 6 lists
   the log lines of a healthy run.

8. Press **Stop** when you are done, or **Restart** to run again, with the same
   config or another one from the list. Ctrl+C on the dashboard stops
   everything.

The rest of this document explains each step in detail and what to do when one
fails.

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
  from the config frame (one slot per schedule character), transmits a short
  beacon once per frame from its replay RAM (section 7.0), and receives its
  receive slots: every slot but the beacon's, or, with `bs_rx_slots`, only the
  slots the schedule marks for receive. The client hunts for the
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
- **A clock plan for the two boards.** Each board's reference is a device
  setting (`clock_ref`) that whoever provisions the boards sets; the full
  setup check (section 2.2) prints it on its `clock` line for each radio. One
  of two plans:
  - **One shared reference.** Feed both boards a common 10 MHz on `CLK IN`
    and confirm the firmware selects the external mux
    (`../../docs/archive/TWO_BOARD_CLOCK_LOCK.md` has the verification procedure).
    The legacy 500 MHz configs were validated this way.
  - **Each board in calibrated hold, the client steered.** With
    `clock_ref = calibrated` on both boards and no shared reference, the
    client tracks the base station's frame timing on its own, but the two
    carriers still sit apart by up to about a ppm, and that offset limits the
    MER (`DEMO_VERIFICATION.md` 9.33, 9.34, 9.44). A config with
    `sync.steer.enable` (every `-steer` config) steers the client's clock onto
    the beacon for the whole run; the demo configs do.
- An RF path between the two boards, cabled or over the air, at the frequency
  your config names.

### 2.2 Shortcut if your host is already provisioned

Run the setup check. If it prints `Ready.`, jump straight to section 3:

```sh
source <your-houdini-venv>/bin/activate
cd <path-to-HoudiniLab>/CC/Sounder
python3 csi_gui/check_setup.py --conf files/<config>.json
```

Each FAIL line names what is missing and the section below that installs it.

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
source <your-houdini-venv>/bin/activate
cmake -B build -DCMAKE_BUILD_TYPE=Release -DSoapySDR_DIR=$VIRTUAL_ENV/share/cmake/SoapySDR
cmake --build build -j
```

`SoapySDR_DIR` points the configure at the SoapySDR the host plugin was
installed with (section 2.4). Without it, a SoapySDR that lives only in the
virtual environment is not found and the configure stops with "SoapySDR
development files not found"; leave it out only when SoapySDR is installed
system wide.

You should end up with `build/sounder`. The GPU beacon correlator is a separate
option, off by default, and the demo does not need it. Leave
`HOUDINI_USE_CUDA` alone unless you are specifically testing that path.

The build also produces the radio-free tests and bench tools (`ctest -N` in
`build` lists them). Run them once after building:

```bash
cd build && ctest
```

They need no radio. A packager who wants only the
sounder can configure with `-DSOUNDER_BUILD_TESTS=OFF`.

### 2.6 Point the demo at your bench

Each config names its topology file in `serial_file`: `houdini-1u`,
`houdini-ul` and `houdini-2ch-decouple` use `files/topology-houdini.json`, and
every `houdini-dualband*` config plus `houdini-r0` use
`files/topology-houdini-dualband.json`. Edit the one your config names and
replace the two addresses with your own. If you use configs from both groups,
edit both files: on the original bench the two files give the same two boards
opposite roles, so check which board is the base station in each. The base station goes under `BaseStations`, the client
under `Clients`:

```json
{
    "BaseStations": { "BS0": { "sdr": [ "<bs-ip>" ] } },
    "Clients":      { "sdr": [ "<ue-ip>" ] }
}
```

Then open the config you plan to run (section 3) and check these fields against
your bench:

- `frequency` and `nco_frequency`: the carrier and the converters' NCO, which
  must match each other. The legacy configs set both to 500 MHz; the
  dual-band configs set both to the sub-6 2425 MHz (2420 MHz in `-40`) and
  move the X-band channels to their own NCO in `channel_nco_frequency`.
- `channel` and `ue_channel` (and, in the multi-channel configs,
  `tx_channel`, `rx_channel`, `ue_tx_channel`, `ue_rx_channel`): which RF
  channels each board uses. The config's `_comment` names the cabling they
  expect.
- `houdini_rx_gain_db` and `houdini_tx_gain_db`, in the dual-band configs
  only: the Houdini gain surface. The receive value is minus the step
  attenuator's attenuation (0 is no attenuation), the transmit value sets the
  DAC output current (0 is the maximum). Each is ONE value per direction,
  written to every channel the radio opens and read back at start, so
  attenuating the sub-6 receive attenuates the X-band receive too. The keys
  apply only in mode V (the dual-band converter plan); a legacy config that
  sets them is refused at load.
- `ue_rx_gain_a` / `ue_tx_gain_a` and the `_b` pair: **these do nothing on a
  Houdini radio, so do not spend bench time sweeping them.** The gain stages
  they name (LNA, PGA, TIA, PAD) belong to the Iris and USRP front ends that
  share this config format.
- `ue_power_ramp` and the `ue_ramp_*` gains are equally inert here: that block
  only runs under the Iris hardware framer, which these configs do not use.
- `ue_tx_advance_ticks`: leave it at 0. It is quantized to the driver's
  384-tick anchor grid, so values under 192 vanish. The fine seating of the
  client pilot comes from `tx_advance` (section 3).

Beyond the gain keys, signal level is set two ways:

- **Physically**, by cabling, filters and attenuation between the two boards.
  Judge a direct cable by the converter range it uses (below); an over the air
  path normally needs no attenuator.
- **Digitally**, by the optional `tx_scale` field. No shipped config sets it,
  which means it is computed automatically to normalize the OFDM peak, and
  that is the right starting point. Set it explicitly only when you need to back
  the transmitted amplitude off.

Judge the level from the receive side rather than guessing at it. Each card's
Spectrum tab shows how much of the converter the pilot uses and counts clipped
samples (section 5.3), and running the sounder with `HOUDINI_CL_RX_DEBUG=1`
makes it print the received RMS and absolute maximum periodically. Samples are
16 bit, so an absolute maximum near 32767 means you are clipping and should
attenuate; an RMS in the low tens means you are close to the noise floor and
the beacon correlation will be marginal.

Leave the frame geometry alone unless you know why you are changing it. The
shipped numbers are load bearing. In the legacy configs 30 slots of 4096
samples is exactly 1 ms per frame at 122.88 MSPS, and `samps_per_slot` must
stay at or below 4096 to fit the FPGA transmit RAM. In the dual-band configs
20 slots of 61440 samples is exactly 10 ms, and every slot starts on the
driver's 384-tick grid; each config's `_numerology_note` shows the arithmetic.

## 3. Choose a config

Each config carries a one-line `_description`, which the dashboard's config
list shows. All of them run one client. Each names its own topology file in
`serial_file` and describes the cabling it expects in `_comment`.

| Config | What it runs |
|---|---|
| `files/houdini-dualband-xw-steer-slots.json` | **The demo.** Sub-6 2425 MHz at 133 RB plus the X-band IF at 4380 MHz at 270 RB (97.2 MHz), 4096 FFT, 30 kHz spacing; the UE's clock steered onto the beacon; the base station receives only its rx slots and removes the carrier offset before the FFT. Needs the slots host plugin (`HOUDINI_SOAPY_ROOT`, `DEMO_BENCH_RUNBOOK.md` A3) |
| `files/houdini-dualband-xw-steer-slots-fe.json` | The demo through an X-band front end held in a static TX/RX state for the session |
| `files/houdini-dualband-xw-steer.json` | The demo's widths and steering, receiving every slot, on the default plugin (the frozen fallback build's config) |
| `files/houdini-dualband-xw-steer-fe.json` | That, through the X-band front end |
| `files/houdini-dualband-steer.json` | Both bands at 133 RB (48 MHz), steered: the fallback when the 97 MHz X-band is too weak |
| `files/houdini-dualband-steer-fe.json` | That fallback through the X-band front end (not yet run on a rig) |
| `files/houdini-dualband-xw.json` | The 97 MHz X-band, unsteered |
| `files/houdini-dualband.json` | R3: both bands at 133 RB, unsteered: the 5G-like numerology the demo builds on |
| `files/houdini-dualband-40.json` | R3 at 40 MHz: both bands at 106 RB with sub-6 centred at 2420 MHz, unsteered; the rollback if the 50 MHz link disappoints |
| `files/houdini-dualband-r3a.json` | The demo numerology on sub-6 only |
| `files/houdini-dualband-r2.json` | Both bands at 256 FFT, 480 kHz spacing |
| `files/houdini-dualband-r1.json` | Sub-6 only at 256 FFT, 480 kHz spacing |
| `files/houdini-r0.json` | The legacy 500 MHz, 64 FFT link on the dual-band roles: the control |
| `files/houdini-1u.json` | Legacy 500 MHz, 64 FFT: channel estimate panels only |
| `files/houdini-ul.json` | Legacy 500 MHz, 64 FFT with an uplink slot: channel estimate **plus** the equalized constellation |
| `files/houdini-2ch-decouple.json` | Legacy 500 MHz, two links on two channels |

The `-fe` configs set `xband_frontend_static`: they need the X-band front-end
boards' roles applied on both nodes (`sudo houdini-role status` exits 0 on
each) and are refused at start without them. The slots configs
(`bs_rx_slots`) need the slots host plugin, selected by exporting
`HOUDINI_SOAPY_ROOT=<its prefix>` before the setup check and the dashboard.

On a new bench, go up the dual-band ladder one rung at a time: `r0` proves the
link and the stack, `r1` the converter clocks and the sub-6 band, `r2` adds the
X-band IF, `r3a` the demo numerology on sub-6, and `houdini-dualband.json` both
bands; the demo configs at the top of the table add the X-band's width, the
clock steering and the slots mode. When a rung fails, the one below it passing
tells you what changed. `r0` runs at 500 MHz, so it is the control only while
no sub-6 bandpass filter sits in the chain: a 2.4 GHz bandpass blocks its
beacon and the client never acquires (`DEMO_VERIFICATION.md` 9.40). On the
legacy bench roles, start with `houdini-1u.json` and move to `houdini-ul.json`
once the channel estimate is clean: the constellation only has something to
draw when the frame carries an uplink data slot.

In the schedule strings, `B` is the beacon, `P` is the pilot, `U` is uplink
data, and `G` is a guard slot. `houdini-ul.json` places the pilot and data at
slots 16 and 18 rather than early in the frame, which keeps them clear of
beacon leakage at the base station; the dual-band configs put them in slots 2
and 3 of their 20-slot frame.

Pilot and data placement is sample exact: the client pads each burst so its
start escapes the driver's 3125 ns scheduling grid, and `tx_advance` in the
config is a bench calibration that seats the burst at its nominal in-slot
position (247 on the legacy one-rate path, 169 on the dual-band converter
path). If you change cabling or the RF path, re-derive it: run with
`HOUDINI_BS_RX_DEBUG=1` across a few restarts and shift `tx_advance` by the
mean `pilot_grid_off` the base station prints (the `_tx_advance_note` of
`houdini-ul.json` and of each dual-band config gives the details).

## 4. Run the demo

You have two ways to start it. Mode A is one command and is the normal choice.
Mode B keeps the sounder in its own terminal, which is better when you are
debugging.

### 4.1 Mode A: the dashboard launches the sounder

```sh
cd <path-to-HoudiniLab>/CC/Sounder
python3 csi_gui/csi_server.py --launch --conf files/<config>.json
```

The backend sets the environment, starts `sounder --view`, and makes up to
four attempts at the cold start, which matters because a radio open can time
out on the first attempt.

Before each attempt it also runs `csi_gui/teardown_framer.py` to release a
framer that a previous run may have left armed (section 8.4). You will see its
output prefixed `[teardown]`, and the sounder's prefixed `[sounder]`.

Two defaults assume one particular layout. If yours differs, override them:

- `--sounder-dir <path-to-HoudiniLab>/CC/Sounder` only to run a different
  checkout from the one `csi_server.py` lives in, which is the default. The
  launcher runs whatever `build/sounder` it finds under this directory, and a
  stale binary looks exactly like the current demo until a log line you expect
  is missing. When in doubt, verify with
  `strings <dir>/build/sounder | grep <a-string-only-the-new-code-logs>`.
- `--venv <your-houdini-venv>` if no environment is activated and the SoapySDR
  virtual environment is not at `~/houdini_test`.

That command on its own is deliberately quiet. It prints the teardown, the
sounder's startup and a `[csi]` datagram counter every five seconds, and
almost nothing per frame. That is the normal amount of output, so do not read
a short log as a sign that something is wrong. Add `--log-dir <dir>` to keep
each start's sounder output in `<dir>/sounder_<UTC>.log`, including the
end-of-run checks it prints at Stop.

If you want the per frame diagnostics instead, export them in the same shell
before launching. Section 7 describes each one:

```sh
export HOUDINI_BS_RX_DEBUG=1 HOUDINI_UE_TX_DEBUG=1 HOUDINI_CSI_R_DEBUG=1
python3 csi_gui/csi_server.py --launch --conf files/<config>.json
```

Every beacon carrier estimate, rather than one in ten, is a config setting
(`"sync": {"cfo": {"log_every": 1}}`, section 7.1), not an export.

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
./build/sounder --view --conf_file files/<config>.json
```

For a slots config (`bs_rx_slots`), point SoapySDR at the slots host plugin
instead of the venv's module directory, as the dashboard does:
`export SOAPY_SDR_ROOT=<slots-plugin-prefix> SOAPY_SDR_PLUGIN_PATH=`.

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
| `--mag-top` | the config's `dashboard_mag_top`, else 90 | Where each card's \|H\| axis starts, in dB; the axis then steps by 10 dB when the trace leaves it (section 5) |
| `--mag-span` | 40 | Height of the \|H\| axis, in dB below its top |
| `--csi-fps` | sounder default (30) | Per antenna stream rate out of the sounder |
| `--launch` | off | Start the sounder in viewing mode on this host at once |
| `--control` | off | Start, Stop, Restart and Check buttons and a config list (the sounder's `files/houdini*.json`) in the page header; section 4.4 |
| `--conf` | none | The config to run (with `--launch` or `--control`), and the one the page's axes and subcarrier layout come from |
| `--sounder-dir` | the checkout `csi_server.py` is in | Which checkout's `build/sounder` runs |
| `--venv` | the activated venv, else `~/houdini_test` | The SoapySDR and Houdini plugin prefix the sounder runs with |
| `--log-dir` | off | Write each start's sounder output to `<dir>/sounder_<UTC>.log` |
| `--record` | `$HOUDINI_CSI_RECORD`, else off | Record every datagram to a new file for `replay_feed.py` (a name that exists is refused, never overwritten) |
| `--record-max-mb` | 2048 | Stop recording at this size |
| `--dest-host` | 127.0.0.1 | Where the sounder sends datagrams, when using `--launch` |
| `--http-host` | 0.0.0.0, or 127.0.0.1 with `--control` | Web server bind address |

`python3 csi_gui/csi_server.py --help` lists every option.

Run the backend on the same host as the sounder unless you have a reason not
to. If you split them, set `--dest-host` to the backend's address and make sure
UDP port 9999 is open between the two.

### 4.4 Start and stop from the page

To restart the experiment from the browser, run the backend with `--control`:

```sh
cd <path-to-HoudiniLab>/CC/Sounder
python3 csi_gui/csi_server.py --control --conf <config>
```

1. Open the dashboard through the SSH port-forward (section 5). With
   `--control` the web server listens on 127.0.0.1 only, so the forward is the
   only way in. Do not add `--http-host 0.0.0.0` on a shared network: anyone
   who can reach the page can then start the radios.
2. The header shows a config list, Start, Restart, Stop, and the sounder's
   state. Nothing runs until you press Start (add `--launch` to start at once).
3. Start and Restart tear down the framers, wait for the boards to release,
   then launch `sounder --view` with the config selected in the list, retrying
   a failed start as `--launch` does. Start does nothing while a sounder
   runs; use Restart. Stop ends the sounder and leaves it stopped.
4. **Check** runs the full setup check (section 0, step 3) against the config in
   the list, including opening both radios, so it only runs while no sounder
   does. It can take up to a minute per radio, and Stop waits for it to finish.
5. The list offers the sounder's own `files/houdini*.json` plus the `--conf`
   you started the backend with. To run another config, copy it there under
   that name.

Ctrl+C on the backend still stops the sounder with it.

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

Each receive antenna gets one card. Once the sounder's channel metadata
arrives the card is titled by its band, for example
`Sub-6 · 2425 MHz (RX ch 0, antenna 0)` or
`X-band · IF 4380 MHz (RX ch 2, antenna 1)`. Its **Channel** tab carries:

1. **\|H\| in dB** across subcarriers. This is the frequency response of the
   channel. Nulls are real multipath fades, not faults. The axis starts at the
   config's `dashboard_mag_top` (or `--mag-top`) and moves in 10 dB steps, at
   most once every 3 seconds, and only when the trace has left the axis or sat
   in its bottom quarter for that whole time; a steady trace never moves it.
   Between steps an `off scale` badge marks a trace outside the axis.
2. **MER over the last 60 seconds**, on a fixed 0 to 40 dB axis, from the
   card's one-second MER. It shows steering, fades and interference as they
   happen; a gap in the line is a stretch with no constellation.
3. **Phase shape** in degrees, on a fixed plus or minus 10 degree axis: each
   frame's phase with its measured delay (a straight line across the band) and
   its common phase removed, averaged over half a second. The removed delay is
   printed beside the title. On a cable what remains is the filters' ripple.
4. **Waterfall of \|H\|**, time running downward. This is the panel that
   shows stability: a steady link draws smooth vertical streaks, and a link
   that keeps re-locking draws horizontal tearing.
5. **Constellation**, equalized uplink data. Only populated when the frame
   carries an uplink data slot (`U` in the schedule). Clean QPSK shows four
   tight clusters.
6. **CIR**, the impulse response of the same H: dB relative to the strongest
   tap on a fixed 0 to -60 dB axis, delay in ns from that tap. A clean cable
   reads as one mainlobe about 2/B wide (the Hann window), not a single tap.

Under the panels the quality line gives the lane's IF (NCO) and transmission
bandwidth (in resource blocks when the tones make whole NR resource blocks),
the MER and EVM (decision directed, averaged over about a second, over the
tones within 8 dB of the median \|H\|), and the delay spread figures with
their threshold and resolution.

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

How long is normal depends on the cadence. The client looks at the beacon only
as often as its tracked clock needs (seconds apart on a steady link;
`sync.resync.residual_ppm` and `sync.resync.sync_tol_samples` set it, section
7.1), so a healthy link is quiet most of the time, and the badge only means
something if it is much longer than the cadence. The page works this out for
itself: it measures the interval between the reports it actually receives and
marks the card quiet at three times that, so the badge keeps its meaning
whatever the cadence. Nothing to configure.

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

### 5.3 The Spectrum tab

Each card has two tabs. **Channel** is everything above. **Spectrum** shows
the received pilot slot in frequency and how much of the converter it uses,
which is where you look when the channel panels are strange and you suspect
the front end rather than the algorithm.

The trace is the pilot slot's **spectrum in dBFS per bin** (the bin width is
in the panel title) on a fixed 0 to -140 dB axis, so a full-scale complex tone
reads 0 dBFS and a level change is a real change, not a rescale. The x axis is
MHz from the lane's NCO on a 10 MHz grid, and two dashed lines mark the
occupied band's edges. A healthy pilot is a flat shelf between the dashed
lines with the floor well below it outside; a spur, an image or a filter edge
shows up where it sits. A value above the top pins to it and lights an
`off scale` badge.

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
CSI view mode: streaming to 127.0.0.1:9999 (64 subcarriers, ~30 fps/ant, rx_conj=1, sym_start=120, timing_fix=1, phase_fix=1)
```

Check each field:

- The destination matches where your backend is listening.
- `rx_conj=1` on Houdini hardware. The receive mixer delivers baseband
  conjugated, and this flag undoes it. If it were wrong, every channel estimate
  would land on the mirror subcarrier and the constellation would scramble.
- `sym_start` is the zero prefix minus half the cyclic prefix: 120 for the
  legacy configs (128 minus half of 16), -112 for the dual-band configs (32
  minus half of 288; negative is valid, the symbol body is read from the
  window start plus the cyclic prefix). Section 7 explains why.
- `timing_fix=1` and `phase_fix=1`, on by default for Houdini.

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
same shell that launches it. The sync knobs (the SNR floor, the beacon carrier
log rate, the tracker and the steering) are config keys instead, in the
config's `sync` block (section 7.1).

| Variable | Default | What it does |
|---|---|---|
| `HOUDINI_CSI_SYM_START` | the zero prefix minus half the cyclic prefix (section 6) | Where the FFT window starts inside a received slot. An integer, or `auto` for the energy edge detector. |
| `HOUDINI_CSI_NO_TIMING_FIX` | unset | Set it to disable the per frame pilot re-alignment. |
| `HOUDINI_CSI_FPS` | 30 | Per antenna datagram rate out of the sounder. |
| `HOUDINI_MAX_FRAME` | from config `max_frame` | Frame count to run. Set large for continuous viewing. |
| `HOUDINI_CSI_UDP` | `127.0.0.1:9999` with `--view` | Where datagrams go, as `host:port`. |
| `HOUDINI_CSI_DUMP` | unset | One shot raw slot and H dump for offline analysis. |
| `HOUDINI_PILOT_HORIZON` | from config `ue_pilot_horizon` (96 in the legacy configs, 10 in the dual-band ones) | How many frames of client bursts are queued ahead of real time. Larger survives slower host loops; every extra frame delays a timing correction reaching the wire. |
| `HOUDINI_BS_RX_DEBUG` | unset | Base station prints its rederivation of the client schedule: `pilot_grid_off` should sit within a few samples of zero and hold steady through a run, and `clamped` (slots placed past the capture's edge) should read 0. |
| `HOUDINI_UE_TX_DEBUG` | unset | Client prints its burst scheduling (frames queued, pad). |
| `HOUDINI_CORE_MAP` | unset | Where the sounder pins its own threads: `main=<core>,recorder=<core>,bsrx=<core>,ue=<core>`, each the base core of that role (thread i on base + i); a role not named keeps the default layout. Logged at start. For CPU isolation experiments. |
| `HOUDINI_TX_CPU_AFFINITY` | unset | `c0,c1,...`: the i-th live client TX stream gets the host plugin's `cpu_affinity=ci`, pinning its pacer worker (SH-427: keep the workers off the cores that take the data NIC's interrupts). Logged per stream at open. |
| `HOUDINI_TX_STREAM_ARGS` | unset | Extra host-plugin arguments for the client's live TX streams, `key=value,key=value` (for example `tx_target_frac=0.75`), each logged at open; `tx_mode`, `tdd` and `mts` are refused. A diagnostic and tuning knob: set it only when the host plugin's owners ask. |
| `HOUDINI_CSI_R_DEBUG` | unset | Recorder prints the per frame pilot re-alignment it chose (`r`, and the blind score behind it), one line per 30 corrections. |
| `HOUDINI_TX_HOST_STATUS` | unset | Set it to log the client host plugin's pacer state (`TX_HOST_STATUS`, `TX_BANK_STATUS`) every link-health period. Cheap; the demo runs with it. |
| `HOUDINI_CNS_DUMP_LOW` | unset | Directory for autopsy dumps of the first few low scoring constellations. The directory must already exist. |

### 7.0 Choosing the beacon waveform

The base station transmits a short burst at the top of every frame and the
client finds it by correlation. Which burst it sends is the config key
`sync.beacon.type` (the top-level `beacon_type` of older configs is still
read, and the two may not disagree). Six are available:

| value | what it is |
| --- | --- |
| `legacy` | The default. Fifteen repeats of a 16 sample training symbol, then two repeats of a 128 sample Gold sequence. |
| `legacy_guard` | The same, with a 32 sample cyclic guard inserted before the Gold field, in the style of an 802.11 long training field. |
| `dot11` | The 802.11a/g/n legacy preamble as the standard defines it: the short training field, then the guard and two long training symbols. |
| `nr` | The 5G NR primary synchronisation signal, then a guard and two repeats of a tracking symbol built from the NR reference sequence. The client finds it on the repeated tracking symbol. |
| `nr_pss` | The same burst as `nr`, sample for sample, but the client finds it the way an NR handset does: a plain matched filter on the primary synchronisation signal, with no repeat check. The log says `threshold form forced to nolag` when this is in effect. |
| `nr_pss_bl` | The band-limited beacon of the dual-band configs: the NR primary synchronisation signal, then a guarded pair of tracking symbols on the central 96 tones, all inside the sub-6 channel filter's plus or minus 24 MHz. |

The first four were measured on the bench, four rounds each with the order
rotated, about 8000 detections apiece; the first five have been run end to end
through the client (`nr_pss` in `DEMO_VERIFICATION.md` 8.154 to 8.162), and
`nr_pss_bl` is the beacon of every dual-band run (section 9 of the record).
The margin figures below were measured with an earlier comparison rule, so
read them as a ranking rather than as absolute numbers. **Timing is the same
for all of them**, within measurement error. What separates them is detection margin: the worst detection
of the run cleared the threshold by 12x for `legacy` and `legacy_guard`, 7x for
`dot11`, and only 2.3x for `nr`.

**On the legacy configs, leave it at `legacy` unless you have a reason.** The
margin is the best of the four, and it is the waveform every legacy
measurement in this repository was taken against. **The dual-band configs need
`nr_pss_bl`:** in mode V the sub-6 lanes are filtered to plus or minus 24 MHz,
so the sounder refuses a wider beacon at load. If you set a value that is not
in the table the client refuses to start rather than falling back, so a typo
cannot quietly leave you on a different beacon than you think.

The one thing the alternatives are better at is the beacon's own frequency
estimate, where `dot11` is about a third more stable. That number is a
secondary reading on the sync panel, not what the client actually tracks
frequency with, so it does not currently justify the margin it costs.

### 7.1 Sync knobs: the `sync` block of the config

These belong to the timing tracker, the beacon detector, its SNR confirm, the
beacon's own frequency estimate and the client's clock steering. They live in
ONE place: a `sync` object in the JSON config, loaded into one validated structure whose
every value is printed at startup with where it came from (`default`, `json`
or `env`). Every default below is a measured value, not a guess, and the run
is expected to be correct with all of them left alone.

Three things to know:

1. The table is GENERATED from the code (`./build/sync_config_schema`) and
   checked by `sync_config_test`, which fails when the committed copy
   differs from the schema, so it cannot drift from what the client actually
   reads. If you edit a knob in the code, regenerate this table.
   `sync.detector.corr_scale` and `corr_scale_init` are the detection bars;
   the legacy top-level `corr_scale` arrays are still read when the block
   does not set them (the first client's value), so no old file needs a new
   key.
2. A key the loader does not know, or a value outside its range, stops the
   client with a message naming the key. A typo cannot quietly leave a knob at
   its default.
3. The `was` column is the environment variable each knob replaces. Those
   variables are OFF by default: a stale export in your shell is reported as
   IGNORED at startup and changes nothing. A config that sets
   `sync.allow_env_overrides` true gets them back, each override logged; a
   number outside a knob's range is then pulled to the nearest bound with a
   note, except for the three knobs whose old readers ignored such a value
   (`beacon.tx_full_scale`, `detector.first_path_window`,
   `detector.first_path_floor_db`), which keep their default with a note.
   Bench sweeps go through the JSON: `run_shape_campaign.sh` merges
   `SYNC_OVERLAY` (a JSON object) into the `sync` block of every config it
   writes. The environment path is kept only for old bench scripts.

Example, the block of the demo config
`files/houdini-dualband-xw-steer-slots.json`:

```json
"sync": {
  "steer":    { "enable": true, "deadband_ppm": 0.12 },
  "beacon":   { "type": "nr_pss_bl", "tx_full_scale": 0.34 },
  "detector": { "pick": "argmax", "min_bar": 0.1 },
  "confirm":  { "snr_floor_db": 25.0 }
}
```

<!-- sync-knob-table:begin (generated by ./build/sync_config_schema; sync_config_test diffs it, do not edit by hand) -->
| key | default | was | range | env out of range | what it does |
| --- | --- | --- | --- | --- | --- |
| `sync.beacon.type` | `legacy` |  |  |  | Which beacon waveform the base station transmits (legacy, legacy_guard, dot11, nr, nr_pss, nr_pss_bl; nr_pss_bl is the band-limited mode-V beacon, AP-79). |
| `sync.beacon.tx_full_scale` | 0.6 | `HOUDINI_BEACON_FS` | 0.001 to 1 | ignored, value kept | Transmit peak of the beacon as a fraction of DAC full scale. 0.6 shipped; lower it to stand in for path loss on a cable. |
| `sync.detector.threshold` | `auto` | `HOUDINI_BEACON_THRESH` | auto, power, xcorr, coherence | refused | Decision statistic: auto picks coherence for a single-copy replica and the normalised cross-correlation otherwise; power is the pre-2026-09 form and the Iris/UHD default. |
| `sync.detector.pfa_per_window` | 0.001 |  | 1e-09 to 0.5 |  | The coherence form's bar when set: the false-alarm probability per search window, turned into a bar by the replica and window lengths (8.163). Unset, corr_scale applies; ignored for the repeated-field forms. |
| `sync.detector.pick` | `first_path` | `HOUDINI_BEACON_PICK` | first_crossing, cluster_refined, argmax, first_path | refused | Which crossing is returned: first_path (the Houdini default), argmax, cluster_refined, or first_crossing (the Iris/UHD default; unsafe on a strong link). |
| `sync.detector.first_path_window` | derived | `HOUDINI_FIRST_PATH_WIN` | -1 to 4095 | ignored, value kept | Samples the first-path search looks back from the peak; -1 (default) means half the replica length. Must stay inside the preamble's self-coherent plateau. A correlator quantity: samples, not scaled with the rate. |
| `sync.detector.first_path_floor_db` | -9 | `HOUDINI_FIRST_PATH_DB` | -30 to 0 | ignored, value kept | How much weaker, in dB of path power, an earlier arrival may be and still be taken as the first path. |
| `sync.detector.first_path_guard` | 0 | `HOUDINI_FIRST_PATH_GUARD` | 0 to 1 | clamped | Samples immediately before the peak the first-path search skips. A beacon between samples splits its peak over two adjacent taps and the earlier one is the SAME arrival, not an earlier one; 1 skips it. Only 0 and 1: 2 loses a genuine two-sample-earlier arrival and 3 a three-sample one (measured). 0, the default, is what every release so far has shipped. |
| `sync.detector.corr_scale` | 10 |  | 0.0001 to 1e+07 |  | Resync detection threshold: the bar is 1 / corr_scale, relaxed by one per retry. Read from the legacy per-client top-level array when absent. |
| `sync.detector.corr_scale_init` | 10 |  | 0.0001 to 1e+07 |  | Acquisition detection threshold (bar 1 / corr_scale_init); defaults to corr_scale. |
| `sync.detector.min_bar` | 0 |  | 0 to 1 |  | The lowest bar the resync retry relaxation (+1 on corr_scale per retry) may reach; 0 = no limit. Set it with a small corr_scale, where +1 per retry would walk the bar into the noise. |
| `sync.detector.corr_threads` | 1 | `SOUNDER_CORR_THREADS` | 1 to 256 | clamped | Threads for the correlator's matched filter. 1 shipped; measured a net loss below ~4 on the rig host. |
| `sync.confirm.snr_floor_db` | 30 | `HOUDINI_SYNC_SNR_DB` | -10 to 80 | clamped | In-window SNR a detection must clear. A property of the link and the waveform: re-derive it when either changes. |
| `sync.cfo.index_guard` | 8 | `HOUDINI_CFO_INDEX_GUARD` | 0 to 64 | clamped | Samples the carrier estimator's windows slide later than the detected end (AP-39). A correlator quantity: samples, not scaled with the rate. |
| `sync.cfo.window_margin` | 0 |  | 0 to 32 |  | Samples shrunk from both ends of each estimator window so neither touches the burst's edge (8.164). A correlator quantity: samples, not scaled with the rate. |
| `sync.cfo.log_every` | 10 | `HOUDINI_CFO_LOG_EVERY` | 1 to 1e+06 | clamped | Print one beacon-CFO log line in this many. |
| `sync.tracker.type` | `alpha_beta` | `HOUDINI_TRACKER` | alpha_beta, kalman | refused | Which estimator tracks the base station frame grid: alpha_beta (shipped) or kalman. |
| `sync.tracker.alpha` | 0.5 | `HOUDINI_GRID_ALPHA` | 0 to 1 | clamped | Fraction of each accepted residual applied to the schedule. |
| `sync.tracker.beta` | 0.1 | `HOUDINI_GRID_BETA` | 0 to 1 | clamped | Fraction of the residual applied to the frame period estimate. |
| `sync.tracker.step_ppm` | 0.5 | `HOUDINI_GRID_STEP_PPM` | 0 to 1000 | clamped | Most one detection may move the period estimate, ppm. 0 disables the limit. |
| `sync.tracker.max_ppm` | 100 | `HOUDINI_GRID_MAX_PPM` | 0.1 to 10000 | clamped | Absolute band the period estimate may occupy either side of nominal, ppm. |
| `sync.tracker.trust_ppm` | 1 | `HOUDINI_GRID_TRUST_PPM` | 0 to 1000 | clamped | How far the tracked period and a fresh acquisition confirm may disagree before the confirm is preferred, ppm. |
| `sync.tracker.kalman.meas_var` | 0.5 | `HOUDINI_KF_MEAS_VAR` | 1e-06 to 1e+06 | clamped | Kalman only: assumed detector scatter variance, samples squared. |
| `sync.tracker.kalman.rate_rw` | 1e-09 | `HOUDINI_KF_RATE_RW` | 0 to 1 | clamped | Kalman only: how fast the frame period wanders, samples squared per frame cubed. |
| `sync.tracker.kalman.innov_gate` | 4 | `HOUDINI_KF_INNOV_GATE` | 0 to 100 | clamped | Kalman only: sigmas an observation may sit from the prediction before it is ignored. 0 disables. |
| `sync.steer.enable` | false | `HOUDINI_CLOCK_STEER` |  | refused | Steer the UE's clock onto the beacon's with CLOCK_ADJ, from the tracked grid rate, inside the sounder. Needs the UE's clock_ref to be calibrated. Off by default; keep it off for A/B and regression runs of the TX path (SH-427): each CLOCK_ADJ RPC holds the device's stream lock about 200 ms, and each push is a rate step the host pacer re-learns. |
| `sync.steer.period_s` | 20 | `HOUDINI_CLOCK_STEER_PERIOD_S` | 2 to 3600 | clamped | Seconds between steering decisions; the tracked rate is averaged over each. A held oscillator drifts slowly, so this need not be short. |
| `sync.steer.gain` | 0.7 |  | 0.05 to 1 |  | Fraction of the averaged offset removed at each push. |
| `sync.steer.deadband_ppm` | 0.06 |  | 0 to 10 |  | Offsets smaller than this are left alone: half the actuator quantum is the floor of what a push can fix. |
| `sync.steer.max_offset` | 30 |  | 0 to 400 |  | Bounded authority: never steer further than this many counts from the calibration point. |
| `sync.steer.max_push` | 2 |  | 1 to 4 |  | Most counts one push may move, so no single frequency step is large. At most 4: the step is fed forward when the push lands, about 0.2 s after the DAC moves (up to 0.4 s when a failed write is read back), so 4 counts (0.5 ppm) leave 12 to 25 samples of grid error, well inside the 246-sample re-sync gate; 50 would leave 150 to 300. |
| `sync.steer.ppm_per_count` | 0.1251 |  | 0.001 to 10 |  | Actuator gain, ppm per CLOCK_ADJ count (magnitude; +1 count raises the UE clock). Measured 0.1251 (AP-48). |
| `sync.steer.keep` | false |  |  |  | Leave the steered code in place when the sounder exits instead of releasing to the calibrated hold. |
| `sync.resync.residual_ppm` | 0.1 | `HOUDINI_SYNC_RESIDUAL_PPM` | 0.0001 to 1000 | clamped | Assumed worst-case clock error after tracking; with sync_tol_samples it sets how often the beacon is looked at. |
| `sync.resync.scatter_tol_us` | 2 | `HOUDINI_SCATTER_TOL_US` | 0.01 to 1000 | clamped | How far a detection may land from the tracked grid and still count as the same beacon, microseconds. |
| `sync.resync.confirm_tol_us` | 5.2083 | `HOUDINI_CONFIRM_TOL_US` | 0.01 to 1000 | clamped | The same tolerance during acquisition. Never applied looser than the tracking gate. |
| `sync.resync.sync_tol_samples` | derived | `HOUDINI_SYNC_TOL_SAMPLES` | 0.5 to 1e+06 | clamped | Timing slack budgeted to drift between looks, samples. Default: a quarter of the OFDM zero prefix. |
| `sync.resync.retry_max` | 100 | `HOUDINI_RESYNC_RETRY_MAX` | 1 to 100000 | clamped | Misses in one resync period before the client logs an exhausted episode. |
| `sync.resync.escalate_episodes` | 2 | `HOUDINI_ESCALATE_EPISODES` | 1 to 1000 | clamped | Consecutive exhausted episodes before the client abandons tracking and re-acquires. |
| `sync.resync.hold_offgrid` | 2 | `HOUDINI_HOLD_OFFGRID` | 1 to 1000 | clamped | Consecutive off-grid detections before the beacon counts as moved. |
| `sync.resync.acq_refine_span` | 200 | `HOUDINI_ACQ_REFINE_SPAN` | 2 to 100000 | clamped | Frames of baseline acquisition wants before it trusts its rate estimate. |
| `sync.resync.acq_max_ppm` | 100 | `HOUDINI_ACQ_MAX_PPM` | 0.1 to 10000 | clamped | Plausibility band applied to a rate that acquisition hands back, ppm. |
| `sync.allow_env_overrides` | false |  |  |  | Whether HOUDINI_* environment variables may override these values (each override is logged; see the policy column). Off by default: sweep through the JSON overlay instead. |
<!-- sync-knob-table:end -->

Diagnostics that dump files or print profiles (`HOUDINI_LOOP_PROFILE`,
`HOUDINI_RX_PROFILE`, `HOUDINI_COALESCE_SLOTS`, `HOUDINI_CSI_NO_PHASE_FIX`,
`HOUDINI_BS_RX_EVERY`, `HOUDINI_DUMP_*`) stay environment variables: they are
not configuration, and a dump switch in a shipped JSON is a trap. Every
`HOUDINI_DUMP_*` file lands under `HOUDINI_DUMP_DIR` (default `/tmp`), so a
bench can keep its dumps out of `/tmp` with one variable. View mode
(`HOUDINI_CSI_UDP`, set by `--view`) writes NO HDF5 file and says so at
startup, so a stray value in your shell cannot silently disable recording.

`sync.resync.retry_max`, `sync.resync.escalate_episodes` and
`sync.resync.hold_offgrid` are the escalation net. Their defaults were tuned
for a client grid that drifts out of tolerance in about a second. With the
clock steered it holds for minutes, so the defaults are conservative by a
wide margin. Change them one at a time against a known good baseline run, and
keep the net rather than removing it: it is what stands between a lost beacon
and a client that flies on stale timing without saying so.

`HOUDINI_CSI_SYM_START` is the one worth understanding. The cyclic prefix guard
is one sided. A window placed early, still inside the prefix, is a valid
circular shift and produces a pure phase ramp that the timing fix recovers. A
window even one sample late pulls the next symbol into the FFT and produces
inter symbol interference that no correction recovers. A window placed
exactly on that cliff edge lets beacon re-lock jitter tip runs into
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
`all 2 radio(s) clear` when they are (section 8.4 explains the topology
argument). Then start your run again.

If the release tool finds nothing and the boards are still held, something
outside your session is holding them, and the board's own server has to be
restarted, on each held board:

```sh
ssh <user>@<radio-ip> 'sudo systemctl restart SoapySDRServer'
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
guessing: escalations print `Re-sync ESCALATION` with a reason, and the base
station prints `UE PILOT LOST` during the outage. Usual causes, most likely
first: the clock plan (section 2.1: no shared reference and no steering, or a
board out of its calibrated hold, which the full setup check's `clock` line
reports), the RF level is wrong so detections fall under the sync SNR floor
(the re-sync lines print the measured SNR; compare it against the config's
`sync.confirm.snr_floor_db`), or the detection bar (`corr_scale`) needs
adjusting for your path.

### 8.3 Channel estimate looks fine but the constellation is a smear

Confirm the config's frame carries an uplink data slot (`U` in the schedule:
`houdini-ul.json` or any dual-band config); without one there is nothing to
equalize. If it does, work through the three causes below in order.

**Cause 1: the FFT window is sitting late and taking in interference.** Try
`HOUDINI_CSI_SYM_START` a few samples lower and watch the constellation
tighten. Section 7 explains the asymmetry: early is recoverable, late is not.

**Cause 2: dropped receive packets.** When a receive packet is lost,
`RadioHoudini::recv` zero-pads the hole so the rest of the window keeps its
true timing. Those zeros are not signal, and an FFT taken across them produces
a wrong channel estimate, which would then equalize every following uplink
data slot until the next clean pilot.

Viewing mode therefore refuses those slots instead of rendering them. When a slot
arrives carrying padded samples it is dropped, that antenna's card dims with a
`stale` badge (section 5.1), and the sounder logs, at most once every five
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
on the dual-band configs, the carrier offset (section 2.1), not at restart
luck.

### 8.4 The sounder will not start, and discovery looks broken

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
the framer abort, clears the transmit RAM, and releases the gate. Without
`--topology` it reads `files/topology-houdini.json`, the legacy configs'
topology, which is the wrong file for a dual-band config (their topology is
`files/topology-houdini-dualband.json`). You can also name radios directly
with `--node <addr>` (repeatable).

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

Check in this order: the clock plan (section 2.1), the RF path is actually
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

## 9. Clean up

Press **Stop** on the page, or Ctrl+C in the backend terminal. In mode A the
backend stops the sounder with SIGINT, the sounder's own stop: it ends its
loop, prints its end-of-run checks and releases a steered clock, and anything
still running after 10 seconds is killed. The sounder also dies with the
backend however the backend ends. In mode B stop the sounder in its own
terminal with Ctrl+C as well. A plain `kill` of the sounder skips its
end-of-run checks and leaves the clock steered; send `kill -INT <pid>`
instead, and `kill -9` only if it has not exited after 10 seconds.

Then confirm nothing is left:

```sh
pgrep -cx sounder      # want 0 when stopped, exactly 1 while running
```

Two sounders against the same radios both stream to the same UDP port, and the
dashboard interleaves their frames, so the display cannot be trusted in either
direction.

After any run that ended abnormally, release the framer before starting again:

```sh
python3 csi_gui/teardown_framer.py --conf files/<config>.json   # or --topology <its serial_file>
```

A framer left armed is the most common reason the next run fails to start, and
the failure does not look like leftover state (section 8.4).
