"""The dashboard's record format: every datagram the sounder sent, with its
arrival time, so a good run can be replayed into the dashboard with no radios
(the canned-data fallback; `replay_feed.py` plays it back). Stdlib only.

File: the 8-byte magic HCSIREC1, then records of [t f64][len u32][payload]:
t is seconds on the recording host's monotonic clock (only the differences
matter), payload the datagram exactly as received."""
import struct
import time

REC_MAGIC = b"HCSIREC1"
REC_HDR = struct.Struct("<dI")


class Recorder:
    """Appends datagrams to a recording until `max_bytes` would be exceeded, then
    stops (once, with a message) and leaves a readable file. Flushes about once a
    second, so a run stopped by a signal loses at most the last second.

    Never overwrites: an existing file (the recording about to be replayed, a
    previous run's) is refused. A file that cannot be created leaves `f` None,
    says why once, and records nothing; it never stops the live dashboard."""

    def __init__(self, path, max_bytes, log=print):
        self.left = max_bytes - len(REC_MAGIC)
        self.log = log
        self.t_flush = time.monotonic()
        self.n = 0
        try:
            self.f = open(path, "xb")
            self.f.write(REC_MAGIC)
        except FileExistsError:
            self.f = None
            log("[csi] NOT recording: %s exists and a recording is never overwritten; "
                "name a new file" % path)
        except OSError as e:
            self.f = None
            log("[csi] NOT recording to %s: %s" % (path, e))

    def write(self, t, data):
        if self.f is None:
            return
        n = REC_HDR.size + len(data)
        if n > self.left:
            self.close()
            self.log("[csi] recording stopped at its size cap after %d datagrams" % self.n)
            return
        try:
            self.f.write(REC_HDR.pack(t, len(data)))
            self.f.write(data)
            if t - self.t_flush >= 1.0:
                self.f.flush()
                self.t_flush = t
        except OSError as e:  # a full disk must not stop the live dashboard
            self.f = None
            self.log("[csi] recording stopped after %d datagrams: %s" % (self.n, e))
            return
        self.left -= n
        self.n += 1

    def close(self):
        if self.f is not None:
            self.f.close()
            self.f = None


def read_recording(path):
    """Yield (t, payload) in order. A record cut short at the end (a run killed
    mid-write) ends the recording instead of raising."""
    with open(path, "rb") as f:
        if f.read(len(REC_MAGIC)) != REC_MAGIC:
            raise ValueError("%s is not a dashboard recording (no %r header)" % (path, REC_MAGIC))
        while True:
            hdr = f.read(REC_HDR.size)
            if len(hdr) < REC_HDR.size:
                return
            t, n = REC_HDR.unpack(hdr)
            data = f.read(n)
            if len(data) < n:
                return
            yield t, data
