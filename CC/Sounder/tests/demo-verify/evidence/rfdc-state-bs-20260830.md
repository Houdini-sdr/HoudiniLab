# BS (168.6.244.21) device state through the sounder init sequence

Captured 2026-08-30 by `tests/demo-verify/bs_init_walk.py` on rig host .64.
Two consecutive runs (`walk_full_run1.jsonl`, `walk_full_run2.jsonl`, on the
rig under `~/demo-verify-evidence/phase2/`) produced identical behavior; the
values below are from run 2. The script replays exactly the calls
Radio.cc:139-198 makes for the BS, in order, and nothing else.

## Identity (getHardwareInfo)

| key | value |
|---|---|
| product / label | Houdini-4 [575524] |
| fpga_version / commit | 1.30 / c88e0b5f (magic RFSC, board rfsoc4x2) |
| fpga_build_date | 2026-08-28T00:50:42 |
| device_version / build | 0.2.2 / **71bcbc6b** (2026-08-28T15:45:43Z) |
| host_version / build | 0.2.2 / c20d7975 (2026-08-28T06:07:04Z) |
| proto_version | 1.0 |
| clock_ref | external |
| tick_rate_hz | 122880000 |
| mtu_bytes | 8192 |
| data_iface | 192.168.5.9 |
| rx_gap_break | 1 (host capability probe) |

FLAG: the v1.30 merge gate (SoapyHoudiniSDR TEST_RESULTS.md, arc/xband-sw)
records device build `c20d7975` on .21; the board now runs `71bcbc6b`, built
9.6 h later the same day. The running device firmware is NOT the gate-recorded
artifact. Cross-check against .22 and against the software lane's deploy
record before treating gate results as covering this exact build.

## State at make() (board defaults, restored every session)

make() resets the converter state: both runs started from the same defaults
even though the previous session had left 122.88 MSPS configured.

| item | value at make |
|---|---|
| RX rate ch1 | 245.76 MSPS (RX_FAB_CLK 30.72 MHz, vld code 3 per construction log) |
| TX rate ch1 | 983.04 MSPS (TX_FAB_CLK 122.88 MHz) |
| RX/TX NCO ch1 | 0 Hz |
| TDD framer | state=idle, gates_held=0 (after teardown ladder) |
| ADC PLL Fs | 1228.80 MHz (boot 3932.16), tiles 0 and 2 |
| DAC PLL Fs | 1966.08 MHz (boot 5898.24), tiles 0 and 2 |
| channels | RX: ADC0.0, ADC0.1, ADC2.0, ADC2.1; TX: DAC0.0, DAC2.0 (TX ch1 = DAC2.0) |

## Per-call observed deltas (identical both runs)

| call | observed delta |
|---|---|
| setSampleRate(RX,1,122.88e6) | rx_rate readback 245.76M -> 122.88M immediately; RX_FAB_CLK stays 30.72 MHz, string moves to "code 2, realized 122880000" |
| setSampleRate(TX,1,122.88e6) | tx_rate readback 983.04M -> 122.88M; TX_FAB_CLK stays 122.88 MHz, "code 0, realized 122880000" |
| setFrequency(RX,1,500e6) | rx_freq 0 -> 499999999.999997 (applied live, pre-stream) |
| setFrequency(TX,1,500e6) | tx_freq 0 -> 499999999.99999535 (applied live, pre-stream) |
| setupStream(RX,[1],local_port=10002,rx_gap_break=1) | HOUDINI_FPGA_TX_PORT "" -> "10002" (FPGA egress source port = 10001 + chan, observed) |
| setupStream(TX,[1],tx_mode=replay) | no change in any readable key |

## Full state after the complete init sequence (run 2, after_stream_tx)

```
EGRESS_STATUS            drop=p0:0,p1:0,p2:0,p3:0;stall_seen=0,stall_evt=0;marked=p0:1,p1:0,p2:0,p3:0
HOUDINI_FPGA_DATA_IFACE  192.168.5.9
HOUDINI_FPGA_RX_PORT     (empty)
HOUDINI_FPGA_TX_PORT     10002
HOUDINI_MTU              8192
HOUDINI_PROTO_VERSION    1.0
HOUDINI_RFDC_INTR_STATUS ADC0.0=0[] ADC0.1=0[] ADC2.0=0[] ADC2.1=0[]
HOUDINI_RX_FIFO_HWM      0
HOUDINI_RX_FRAME_WORDS   1016
HOUDINI_RX_PAYLOAD_BYTES 8144
HOUDINI_RX_TARGET_LATENCY 0
HOUDINI_TICK_RATE        122880000
RX_BANK_STATUS           ch0..ch3: gated=0,aborts=0,hwm=0
RX_FAB_CLK               30720000 Hz (code 2, realized 122880000 Hz; commanded code 2)
TDD_ARM                  no arm attempted
TDD_STAT                 state=idle epoch_late=0 ... gates_held=0 ... sched_abut_rx_tx=1 ... ptr=8
TX_BANK_STATUS           ch0,ch1: all counters 0, free=64,cap=64,aempty=1
TX_BUF_WATERMARK         ch0,ch1: high=48,low=16
TX_EMIT_LEAD             ch0:0;ch1:0
TX_FAB_CLK               122880000 Hz (code 0, realized 122880000 Hz; commanded code 0)
rx_rate_ch1 / tx_rate_ch1   122880000.0 / 122880000.0
rx_freq_ch1 / tx_freq_ch1   499999999.999997 / 499999999.99999535
```

## Incidental observations

1. Run 2's make() logged a latched RFDC interrupt from run 1's teardown:
   `DAC0.0 IntrStatus=0x80000008 [FIFOMRGNIND_UF FIFO_OVR]` on a DAC the
   session never used (TX ch1 is DAC2.0). Recorded; compare with the SH-335
   incident notes where the DAC FIFO interrupt was called a bystander.
2. `sched_abut_rx_tx=1` and `ptr=8` persist across teardown and both runs:
   leftover stickies from an earlier armed schedule. `TDD_POS` values
   (pos_tick=18266, pos_frame=77570826) are frozen while idle.
3. Reading a write-only key (`TX_REPLAY_RANGE`) warns "unknown key" and
   returns empty, no exception: the silent no-op trap confirmed live.
4. The "realized" vs "commanded" wording in the FAB_CLK strings is recorded
   verbatim; its interpretation is UNVERIFIED here.
5. QMC, mixer detail, and interp/decim codes beyond the rate strings are not
   exposed through the settings surface; an on-board register peek is the
   optional independent leg if needed.
