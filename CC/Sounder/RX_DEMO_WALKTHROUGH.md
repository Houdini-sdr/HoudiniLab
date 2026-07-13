# Running the max rate RX demo capture, step by step

This walkthrough is for someone who has never used this software. It
goes from an empty machine to a finished analysis: install the
dependencies, build the tool, give the binary the permissions it
needs, run the zero copy demo capture (8 seconds of raw RF samples at
1.96608 GSPS, about 59 GB of data), verify the result, and read the
file in Python. Every step shows the exact command and what you should
see. Reference material lives in `RX_RECORDER.md` (all config keys)
and `../../docs/RX_MAX_RATE.md` (design and measurements); you do not
need either to follow this page.

Validated result for this exact recipe (2026-07-12, rig B): 15000 of
15000 slots recorded, 99.998 percent of samples captured, and the few
lost samples exactly listed in the file's gap table.

## 1. What the demo does

- `rx-recorder` is a command line tool. It opens the Houdini SDR
  (an RFSoC board on the bench network), sets the sample rate, streams
  raw I/Q samples over the 100 GbE link, and writes them to one HDF5
  file.
- At the demo rate the stream is 7.86 GB/s, which is faster than the
  disk. The tool therefore records into a large RAM buffer during the
  capture and writes the file to disk afterwards. You will see these
  two phases in the console output.
- Samples arrive in "slots" of 1,048,576 samples (4 MiB). One slot is
  one row of the HDF5 dataset. An 8 second capture is 15000 slots.
- If any samples are lost in transport, the tool inserts placeholder
  zeros so the file timeline stays correct, and records exactly which
  sample ranges are untrusted in a gap table inside the file. You will
  verify this table in section 6.

## 2. Install everything (from nothing)

### 2.1 The machines involved

- The capture host: a Linux machine with a 100 GbE NIC cabled to the
  radio's data port. On our bench this is rig B,
  `ssh houdini@168.6.244.64`.
- The radio: the RFSoC at `168.6.244.22`. It must be powered, running
  the Houdini server, and on the validated firmware (fpga v1.20,
  device and host plugin v0.2.0). Board bring-up is owned by the
  SoapyHoudiniSDR and Houdini-Streaming projects and is not covered
  here; if you did not set the board up yourself, ask whoever did.

### 2.2 Shortcut if you are on rig B

Rig B is already fully provisioned. Verify with the four checks below
and, if they all pass, jump straight to section 3:

```sh
ls ~/repos/HoudiniLab-rx/CC/Sounder/build/rx-recorder     # binary exists
getcap ~/repos/HoudiniLab-rx/CC/Sounder/build/rx-recorder # caps present (2.6)
ip link show enp1s0f1np1 | head -1                        # mtu 3498 (2.7)
source ~/houdini_test/bin/activate && python -c "import h5py, numpy"
```

### 2.3 System packages

On Ubuntu (22.04 or 24.04, x86_64 or aarch64), install the build tools
and libraries:

```sh
sudo apt install build-essential cmake git \
    libgflags-dev libhdf5-dev \
    ethtool libcap2-bin python3-venv python3-pip
```

What each is for:

- `build-essential`, `cmake`, `git`: compiler and build system.
- `libgflags-dev`: command line flag parsing (build dependency).
- `libhdf5-dev`: the HDF5 library, C and C++ (the capture file
  format). Version 1.10 or newer.
- `ethtool`: reading NIC statistics when you need to debug loss.
- `libcap2-bin`: provides `setcap` and `getcap` for section 2.6.
- `python3-venv`, `python3-pip`: the analysis environment (2.8).

The JSON parser (nlohmann) is bundled with the source; nothing to
install for it.

### 2.4 SoapySDR and the Houdini driver plugin

`rx-recorder` talks to the radio through SoapySDR (a vendor neutral
SDR API, version 0.7 or newer) plus the Houdini host plugin (the
driver that knows this specific radio). The plugin is built and
installed from the SoapyHoudiniSDR repository; follow that repo's host
README. On our rigs the whole stack is installed into a Python
virtual environment prefix at `~/houdini_test`.

Verify the stack is present (rig style install):

```sh
source ~/houdini_test/bin/activate
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
SoapySDRUtil --info          # prints versions and the module list
ls $SOAPY_SDR_PLUGIN_PATH    # must contain a Houdini .so module
```

If `SoapySDRUtil` is missing or the module directory has no Houdini
entry, stop here and complete the SoapyHoudiniSDR host install first.
Nothing in this walkthrough can work without it.

### 2.5 Get and build rx-recorder

Clone the HoudiniLab repository (on rig B the checkout already exists
at `~/repos/HoudiniLab-rx`; skip the clone there):

```sh
git clone <your HoudiniLab remote> ~/repos/HoudiniLab
cd ~/repos/HoudiniLab/CC/Sounder
```

Configure and build. The `-DCMAKE_PREFIX_PATH` line tells CMake to
find SoapySDR inside the venv prefix; keep the venv active from 2.4:

```sh
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$VIRTUAL_ENV"
cd ..
cmake --build build --target rx-recorder -j$(nproc)
ls -l build/rx-recorder
```

Notes:

- Build only the `rx-recorder` target as shown. The full `sounder`
  application has extra dependencies (muFFT) that this demo does not
  need.
- If CMake reports "SoapySDR development files not found", the venv
  was not active or the prefix path was wrong; redo 2.4 and the
  configure step.
- If CMake reports missing HDF5 or gflags, revisit 2.3.

### 2.6 Give the binary its permissions (needs root, once per build)

The zero copy network path bypasses the kernel's normal socket stack,
so the binary needs five Linux capabilities. Without them the demo
refuses to start. Ask someone with sudo to run:

```sh
sudo setcap cap_net_raw,cap_net_admin,cap_bpf,cap_ipc_lock,cap_sys_nice+ep \
    build/rx-recorder
```

What each capability is for:

- `cap_net_raw` and `cap_bpf`: create and attach the AF_XDP zero copy
  socket on the NIC queue.
- `cap_net_admin`: NIC administration during the attach.
- `cap_ipc_lock`: pin the shared packet memory so the NIC can DMA
  into it.
- `cap_sys_nice`: let the driver's receive worker take real time
  scheduling priority.

Verify:

```sh
getcap build/rx-recorder
```

Expected output (order may differ):

```
build/rx-recorder cap_net_admin,cap_net_raw,cap_ipc_lock,cap_sys_nice,cap_bpf=ep
```

Two important gotchas:

- Rebuilding creates a new binary file, and that silently loses the
  capabilities. After ANY rebuild, run this section again.
- A binary that carries capabilities is loaded in "secure exec" mode:
  the dynamic linker ignores `LD_LIBRARY_PATH`. The build embeds the
  venv library path into the binary, and `SOAPY_SDR_PLUGIN_PATH` is
  read by SoapySDR itself, so the normal run recipe works. Just do not
  rely on `LD_LIBRARY_PATH` tricks with this binary.

### 2.7 Set the data NIC packet size (needs root, once per boot)

The zero copy path requires the data interface to use a specific
packet size limit (MTU 3498; on rig B the interface is
`enp1s0f1np1`). Check:

```sh
ip link show enp1s0f1np1 | head -1
```

The line must contain `mtu 3498`. If it shows another value (for
example 9000), ask someone with sudo to run:

```sh
sudo ip link set enp1s0f1np1 mtu 3498
```

Warning for shared rigs: other tools (for example the correlator work)
expect MTU 9000 on this interface. If you change it, tell the other
users, and restore it when you are done:

```sh
sudo ip link set enp1s0f1np1 mtu 9000
```

If the MTU is wrong the demo does not damage anything; it refuses to
start with a clear error (see section 8).

### 2.8 Python environment for analysis

The verification and reading steps (sections 6 and 7) need Python
with three packages. On the rigs, reuse the existing environment
(`source ~/houdini_test/bin/activate`; it already has them). On a
fresh machine:

```sh
python3 -m venv ~/rxdemo-venv
source ~/rxdemo-venv/bin/activate
pip install numpy h5py matplotlib
```

## 3. Before every run

Do these three quick checks each time, even on a provisioned rig.

Check 1: the rig must be free. Only one program may stream from the
radio at a time, and another CPU heavy job on the host will cause
sample loss:

```sh
ps aux | grep -iE "soapy|pytest|rx-recorder|correlator" | grep -v grep
```

If this prints anything, stop and come back when the rig is free.

Check 2: enough disk and RAM. Each 8 second run writes about 59 GB
into `~/rx_logs` and uses about 65 GB of RAM during the capture:

```sh
df -h ~ | tail -1
free -g
```

You want at least 100 GB free disk and at least 80 GB free RAM. Do not
start other large programs while the capture runs.

Check 3: environment set in THIS shell (both lines, every new shell):

```sh
source ~/houdini_test/bin/activate
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
```

## 4. Run the capture

Step 1. Go to the tool directory:

```sh
cd ~/repos/HoudiniLab-rx/CC/Sounder     # or your clone from 2.5
```

Step 2. Start the capture with the demo configuration:

```sh
./build/rx-recorder -conf_file files/rx-record-demo.json -storepath ~/rx_logs
```

Step 3. Watch the console. In order you should see:

1. `Opening device: ...` and several `stepping RX_FAB_CLK ... Hz`
   lines. The tool is raising the radio's internal clock in steps to
   reach the demo rate. This takes tens of seconds; be patient.
2. `Capture plan: 8.000 s @ 1966.080 MSPS ...` followed by
   `read path: readstream (sample-exact extents)`.
3. A pause of up to half a minute with no output. This is the 64 GiB
   RAM buffer being allocated and cleared. Normal.
4. `[INFO] xsk: zero-copy RX engaged on enp1s0f1np1 q19 (umem
   1073741824 B, 4096-B chunks)`. This line is the proof that the fast
   network path is active. If you do not see it, the run stopped with
   an error; see section 8.
5. Eight seconds of capture (usually silent; a warning here means
   samples are being lost, the run still completes and accounts them).
6. `Capture done (15000 slots); draining queued slots to disk ...`
   with progress lines like `Saving HD5F: 2000 frames saved`. The
   drain takes roughly one minute.
7. The `==== rx-recorder summary ====` block, then the program exits.

The whole run takes a few minutes end to end.

## 5. Read the summary

A good run looks like this:

```
==== rx-recorder summary ====
Recorded slots      : 15000 / 15000 (8.000 s)
Dropped slots (host): 0
Read path           : readstream
Stream gaps         : 1 (326480 placeholder samples inserted, exact extents)
Untrusted extents   : 1 (see /Data/Gaps)
Overflows           : 0 readStream / 0 status events
Read timeouts       : 0, other status events: 0
Output file         : /home/houdini/rx_logs/rx_record_20260712_180254.h5
```

What each line means:

- `Recorded slots 15000 / 15000`: every slot of the 8 seconds was
  written. Anything less means the run was cut short.
- `Dropped slots (host): 0`: the disk writer kept up. Must be 0.
- `Stream gaps`: how many separate loss events happened, and how many
  placeholder zero samples were inserted to keep the timeline correct.
  One or two small gaps in the first moments of a capture are normal
  (stream startup). 326480 samples out of 15.7 billion is 0.002
  percent.
- `Untrusted extents`: the number of rows in the file's gap table.

Rule of thumb: the run is good when slots are complete, host drops are
0, and the inserted placeholder count is below about 0.1 percent of
the total (15.7 billion samples for 8 seconds).

## 6. Verify the file

Run the inspector on the file the summary printed (still in the same
shell, so the venv is active):

```sh
python tools/inspect_rx_record.py ~/rx_logs/rx_record_20260712_180254.h5
```

It prints the file's metadata, then a trust report:

```
==== trust report ====
  slots recorded        : 15000 (15728640000 samples, 8.000 s)
  write errors          : 0
  read path             : readstream (sample-exact extents)
  untrusted samples     : 326480 (0.00 %)
  gap extents           : 1
  ...
  loss by second (untrusted fraction):
    s0: 0.0 %
==== payload sanity ====
  peak |int16|          : 88 (-51.4 dBFS)
  mean |int16|          : 8.8
  zero fraction         : 5.03 %
```

How to read it:

- `untrusted samples` is the total number of samples you must not use,
  as an absolute count and a percentage. Near zero is good.
- `loss by second` shows when the loss happened. Loss only in `s0`
  (the first second) is the normal startup transient.
- `payload sanity` proves the file contains live signal rather than
  zeros or a stuck value. With nothing transmitting you should see a
  noise floor around minus 50 dBFS, as above.

If the numbers are bad (percent level loss), the file is still honest;
diagnose the loss mechanism with:

```sh
python tools/gap_forensics.py ~/rx_logs/<file>.h5
```

and compare its fingerprints against `../../docs/RX_MAX_RATE.md`
(section "V4 root-cause"). Also check that nothing else was running
(section 3, check 1) before suspecting the software.

## 7. Read the data yourself in Python

Use the analysis environment from 2.8. Below is a complete example;
paste it into `python` or save it as a script. Replace the file name.

```python
import h5py
import numpy as np

path = "/home/houdini/rx_logs/rx_record_20260712_180254.h5"
f = h5py.File(path, "r")
data = f["Data"]

# Attributes arrive as tiny arrays; this helper unwraps them.
def attr(name):
    v = np.asarray(data.attrs[name]).ravel()
    return v[0] if v.size == 1 else data.attrs[name]

rate = float(attr("RATE"))              # samples per second
slot_len = int(attr("SLOT_SAMP_LEN"))   # samples per slot (row)
print("rate:", rate, "Hz, samples per slot:", slot_len)
print("Samples dataset shape:", data["Samples"].shape)
# Shape is (slots, 1, 1, channels, 2*slot_len). The last axis is
# interleaved I,Q int16: I0, Q0, I1, Q1, ...
# Note: the first dimension grows in blocks of 2000 rows, so it can
# read LARGER than the capture. The true count is SLOTS_RECORDED;
# rows beyond it are unwritten zeros.
n_slots = int(attr("SLOTS_RECORDED"))
print("slots actually recorded:", n_slots)

# The file is ~59 GB. Never read it all at once; read one slot
# (4 MiB) at a time.
slot_index = 100
row = data["Samples"][slot_index, 0, 0, 0, :]

# Convert interleaved int16 to complex numbers:
x = row[0::2].astype(np.float32) + 1j * row[1::2].astype(np.float32)
print("slot", slot_index, "mean power:",
      float(np.mean(np.abs(x) ** 2)))

# Absolute time. The file promises: sample k (global index across the
# whole file) was digitized at FIRST_SAMPLE_TIME_NS + k * 1e9 / RATE.
# The global index of sample j inside slot s is  k = s*slot_len + j.
# Time attributes are stored as strings to keep full precision.
first_ns = int(attr("FIRST_SAMPLE_TIME_NS"))
k = slot_index * slot_len
print("slot", slot_index, "starts at t =",
      first_ns + k * 1e9 / rate, "ns (hardware timebase)")

# The gap table: which sample ranges are placeholder zeros, not RF.
gaps = data["Gaps"][:]      # columns: start_sample, n_samples,
                            #          start_time_ns, cause
print(attr("GAP_COLUMNS"))
for start, n, t_ns, cause in gaps:
    print(f"untrusted: samples [{start}, {start + n}) cause={cause}")
# Skip these ranges in any analysis. Everything outside them is
# genuine, gap free, evenly spaced RF data.

f.close()
```

Optional: a quick look at the spectrum of one slot (matplotlib is in
the venv; over ssh, save to a file instead of showing a window):

```python
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

spec = np.fft.fftshift(np.fft.fft(x[:262144]))
freqs = np.fft.fftshift(np.fft.fftfreq(262144, d=1.0 / rate))
plt.plot(freqs / 1e6, 20 * np.log10(np.abs(spec) + 1e-9))
plt.xlabel("baseband MHz")
plt.ylabel("dB (uncalibrated)")
plt.savefig("slot_spectrum.png", dpi=120)
print("wrote slot_spectrum.png")
```

## 8. If something goes wrong

- Error mentions `rx_xsk=require` and the MTU, and the run stops
  before capturing: the NIC MTU is not 3498. Fix per 2.7. The tool
  checked and refused without touching anything; this is the intended
  fail loud behavior.
- Error about missing capabilities, permission denied, or the same MTU
  error right after a rebuild: the capability bundle is missing from
  the (new) binary. Fix per 2.6.
- `SoapySDR::Device::make returned null` or no Houdini device found:
  the plugin path is not set in this shell (section 3, check 3), or
  the Houdini plugin is not installed (2.4), or the board at
  `168.6.244.22` is down.
- CMake or build errors: see the notes at the end of 2.5.
- Error containing "a stream is open": something else is streaming
  from the radio. See section 3, check 1. If you just aborted a run,
  the server side needs a few seconds to clean up; wait 10 seconds
  and retry.
- Summary shows large `Stream gaps` (percent level): something
  competed for the host or the link during capture. Verify the rig was
  quiet, then run `tools/gap_forensics.py` on the file and match the
  fingerprints in `../../docs/RX_MAX_RATE.md`.
- The program was interrupted (Ctrl-C): the file is still valid up to
  the point of interruption, and the summary says so. Partial files
  are safe to inspect the same way.

## 9. Clean up

Capture files are large. When a file has served its purpose:

```sh
ls -lh ~/rx_logs/
rm ~/rx_logs/rx_record_<stamp>.h5
```

If you changed the NIC MTU on a shared rig, restore it (2.7).
