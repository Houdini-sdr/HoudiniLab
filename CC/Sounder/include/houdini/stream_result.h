/**
 * @file houdini/stream_result.h
 * @brief What a host-plugin stream call's return means to the loop that made it.
 *
 * The SoapyHoudiniSDR 0.4.0 contract (HOUDINI_PROTOCOL 6): a stream fault (a worker exception, a failed socket
 * setup, an unstamped RX packet, an xsk out-of-order completion) ENDS the
 * stream. readStream first delivers what the worker published before the
 * fault, then returns SOAPY_SDR_STREAM_ERROR at once, without waiting out its
 * timeout, on every call until a fresh activation; after a TX worker fault
 * writeStream does the same. The plugin queues exactly one STREAM_ERROR status
 * event for the fault. TIMEOUT means only an empty ring on a healthy stream.
 *
 * The plugin's other STREAM_ERROR returns are caller errors (a null buffer for
 * any lane, a read while direct buffers are held, a write racing a deactivate),
 * which the sounder's loops do not make. So on a Houdini host stream STREAM_ERROR always means the stream is
 * over, and a loop that retried on it would spin at full speed for the rest of
 * the run, logging, and record nothing.
 *
 * So the Houdini radio does not return it: RadioHoudini throws Ended from the
 * read or write, and no loop can retry. The receiver threads' catch stops the
 * run with the fault (Config::stopOnFault), and the run exits with a failure
 * status. A replay TX load is not a host stream (it writes through to the
 * device) and keeps its return code. The rule is this plugin's only: native
 * UHD, for one, maps a late command to STREAM_ERROR.
 */
#pragma once

#include <SoapySDR/Constants.h>
#include <SoapySDR/Errors.h>

#include <stdexcept>
#include <string>

namespace houdini {
namespace stream {

/// Whether a readStream / writeStream return says the stream has ended.
inline bool ended(int r) { return r == SOAPY_SDR_STREAM_ERROR; }

/// A read that delivered samples without HAS_TIME. The plugin stamps every
/// packet and ends the stream on one without the stamp, so this is a broken
/// contract, not a read to splice untimed.
inline bool unstamped(int r, int flags) { return r > 0 && (flags & SOAPY_SDR_HAS_TIME) == 0; }

/// The Houdini radio's report of an ended stream: thrown, not returned.
class Ended : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// Throws Ended(`what`) when `r` says the stream has ended; else nothing.
inline void throwIfEnded(int r, const std::string& what) {
  if (ended(r)) throw Ended(what);
}

/// The one result of a write split over per-channel TX streams (SH-235): the
/// first write that failed or came up short, so the caller's BAD Write check
/// fires, except that an ended stream on ANY channel wins, since that is the
/// one the caller must end the run on. `so_far` starts at `samples`.
inline int mergeWrite(int so_far, int r, int samples) {
  if (r == samples || ended(so_far)) return so_far;
  if (ended(r) || so_far == samples) return r;
  return so_far;
}

}  // namespace stream
}  // namespace houdini
