#!/usr/bin/env python3
"""Bottom-bits liveness of RX samples: histogram of value & 3 over a capture.
No TDD involved: plain rates + RX stream + continuous activate on the BS."""
import sys
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_CS16, SOAPY_SDR_RX, SOAPY_SDR_TX

# SoapyRemote's `timeout` device arg is MICROSECONDS and it bounds the make()
# RPC, not the stream. Measured on this bench: a COLD make (the server holds no
# live device instance, so construction runs the full RFDC bring-up) takes
# 3.34 s, a WARM one 0.34 s, so the long-standing 1000000 (= 1 s) sat INSIDE the
# normal spread. A `SoapyRPCUnpacker::recv() TIMEOUT` on make is that, NOT an
# unresponsive server: three were misread as a session wedge in one session
# before it was measured. readStream's timeoutUs is a different thing and stays.
RPC_TIMEOUT_US = "30000000"

ip = sys.argv[1] if len(sys.argv) > 1 else "168.6.244.21"
dev = SoapySDR.Device(dict(driver="houdinisdr", remote="tcp://%s:55132" % ip,
                           timeout=RPC_TIMEOUT_US))
dev.setSampleRate(SOAPY_SDR_RX, 1, 122.88e6)
dev.setFrequency(SOAPY_SDR_RX, 1, 500e6)
rxs = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [1],
                      dict(local_port="10002", rx_gap_break="1"))
dev.activateStream(rxs)
buf = np.zeros(2 * 8192, dtype=np.int16)
vals = []
for _ in range(16):
    sr = dev.readStream(rxs, [buf], 8192, timeoutUs=1000000)
    if sr.ret > 0:
        vals.append(buf[:sr.ret * 2].copy())
dev.deactivateStream(rxs)
dev.closeStream(rxs)
v = np.concatenate(vals)
n = v.size
lsb2 = np.bincount((v & 3).astype(np.int64), minlength=4)
print("samples(I+Q values): %d  rms=%.2f  min=%d max=%d" %
      (n, float(np.sqrt(np.mean(v.astype(np.float64) ** 2))), v.min(), v.max()))
print("value & 3 histogram: 0:%d (%.1f%%)  1:%d (%.1f%%)  2:%d (%.1f%%)  3:%d (%.1f%%)"
      % (lsb2[0], 100.0 * lsb2[0] / n, lsb2[1], 100.0 * lsb2[1] / n,
         lsb2[2], 100.0 * lsb2[2] / n, lsb2[3], 100.0 * lsb2[3] / n))
print("bit0 alive: %s   bit1 alive: %s"
      % (bool((v & 1).any()), bool((v & 2).any())))
uniq = np.unique(v)
print("distinct values: %d  first few: %s" % (uniq.size, uniq[:12]))
