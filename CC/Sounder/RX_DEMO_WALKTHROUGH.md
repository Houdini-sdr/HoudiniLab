# Running the max rate RX demo capture, step by step

This walkthrough is for someone who has never used this software. It
explains how to run the zero copy demo capture (8 seconds of raw RF
samples at 1.96608 GSPS, about 59 GB of data) and how to check and read
the file it produces. Every step shows the exact command and what you
should see. Reference material lives in `RX_RECORDER.md` (all config
keys) and `../../docs/RX_MAX_RATE.md` (design and measurements); you do
not need either to follow this page.

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
  verify this table in step 5.

## 2. Before you start

### 2.1 The machines involved

- You run everything on the rig B host: `ssh houdini@168.6.244.64`.
- The radio is the RFSoC at `168.6.244.22`. It must be powered, running
  the Houdini server, and on the validated firmware (fpga v1.20,
  device and host plugin v0.2.0). If you did not set the board up
  yourself, ask whoever did before continuing.

### 2.2 Make sure the rig is free

Only one program may stream from the radio at a time, and another
CPU heavy job on the host will cause sample loss. Check:

```sh
ps aux | grep -iE "soapy|pytest|rx-recorder|correlator" | grep -v grep
```

If this prints anything, the rig is busy. Stop, and come back when it
is free.

### 2.3 Find the software

The built tool lives in the application worktree on the rig host:

```sh
cd ~/repos/HoudiniLab-rx/CC/Sounder
ls build/rx-recorder
```

If the binary is missing or you were told to rebuild:

```sh
source ~/houdini_test/bin/activate
cmake --build build --target rx-recorder -j$(nproc)
```

Important: rebuilding creates a new binary file, and that loses the
special capabilities from step 2.4. After ANY rebuild, step 2.4 must be
done again.

### 2.4 Check the capability bundle (one time, needs root)

The zero copy network path needs a few Linux capabilities on the
binary. Check they are present:

```sh
getcap build/rx-recorder
```

Expected output (order may differ):

```
build/rx-recorder cap_net_admin,cap_net_raw,cap_ipc_lock,cap_sys_nice,cap_bpf=ep
```

If `getcap` prints nothing, ask someone with sudo to run:

```sh
sudo setcap cap_net_raw,cap_net_admin,cap_bpf,cap_ipc_lock,cap_sys_nice+ep \
    build/rx-recorder
```

### 2.5 Check the network card MTU (one time, needs root)

The zero copy path requires the data network interface to use a
specific packet size limit (MTU 3498). Check:

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
users, and restore it afterwards:

```sh
sudo ip link set enp1s0f1np1 mtu 9000
```

If the MTU is wrong the demo does not damage anything; it refuses to
start with a clear error (see section 7).

### 2.6 Check free disk and memory

Each 8 second run writes about 59 GB into `~/rx_logs` and uses about
65 GB of RAM during the capture:

```sh
df -h ~ | tail -1
free -g
```

You want at least 100 GB free disk and at least 80 GB free RAM. Do not
start other large programs while the capture runs.

## 3. Run the capture

Step 1. Log in and go to the tool directory:

```sh
ssh houdini@168.6.244.64
cd ~/repos/HoudiniLab-rx/CC/Sounder
```

Step 2. Activate the Python environment and tell the tool where the
Houdini driver plugin lives. Both lines are needed in every new shell:

```sh
source ~/houdini_test/bin/activate
export SOAPY_SDR_PLUGIN_PATH=$VIRTUAL_ENV/lib/SoapySDR/modules0.8-3
```

Step 3. Start the capture with the demo configuration:

```sh
./build/rx-recorder -conf_file files/rx-record-demo.json -storepath ~/rx_logs
```

Step 4. Watch the console. In order you should see:

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
   an error; see section 7.
5. Eight seconds of capture (usually silent; a warning here means
   samples are being lost, the run still completes and accounts them).
6. `Capture done (15000 slots); draining queued slots to disk ...`
   with progress lines like `Saving HD5F: 2000 frames saved`. The
   drain takes roughly one minute.
7. The `==== rx-recorder summary ====` block, then the program exits.

The whole run takes a few minutes end to end.

## 4. Read the summary

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

## 5. Verify the file

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
(step 2.2) before suspecting the software.

## 6. Read the data yourself in Python

The venv already has `h5py` and `numpy`. Below is a complete example;
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

## 7. If something goes wrong

- Error mentions `rx_xsk=require` and the MTU, and the run stops
  before capturing: the NIC MTU is not 3498. Fix per step 2.5. The
  tool checked and refused without touching anything; this is the
  intended fail loud behavior.
- Error about missing capabilities, permission denied, or the same MTU
  error right after a rebuild: the capability bundle is missing from
  the (new) binary. Fix per step 2.4.
- `SoapySDR::Device::make returned null` or no Houdini device found:
  the plugin path is not set in this shell (step 3, both lines), or
  the board at `168.6.244.22` is down.
- Error containing "a stream is open": something else is streaming
  from the radio. See step 2.2. If you just aborted a run, the server
  side needs a few seconds to clean up; wait 10 seconds and retry.
- Summary shows large `Stream gaps` (percent level): something
  competed for the host or the link during capture. Verify the rig was
  quiet, then run `tools/gap_forensics.py` on the file and match the
  fingerprints in `../../docs/RX_MAX_RATE.md`.
- The program was interrupted (Ctrl-C): the file is still valid up to
  the point of interruption, and the summary says so. Partial files
  are safe to inspect the same way.

## 8. Clean up

Capture files are large. When a file has served its purpose:

```sh
ls -lh ~/rx_logs/
rm ~/rx_logs/rx_record_<stamp>.h5
```

If you changed the NIC MTU on a shared rig, restore it (step 2.5).
