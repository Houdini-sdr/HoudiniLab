#!/usr/bin/env python3
"""Release the TDD framer on the demo's radios, so the next run can start clean.

A sounder run that is killed rather than stopped can leave the base station
framer armed and the transmit RAM loaded. The next run then fails to start, and
the failure looks like a discovery problem rather than leftover state. This
script clears that: for each radio in the topology it issues the framer abort,
clears the transmit RAM, and releases the gate.

`csi_server.py --launch` runs this before each sounder attempt. You can also run
it by hand after any run that ended abnormally:

    python3 csi_gui/teardown_framer.py                     # radios from the topology file
    python3 csi_gui/teardown_framer.py --node 10.0.0.5     # or name them explicitly

Exit status is 0 only when every radio was torn down. Any radio that could not
be opened or torn down makes the exit status non-zero, so a caller can tell the
difference between "cleared" and "could not clear".

This opens a connection to each radio, so it IS a device-touching operation. Do
not run it against boards someone else is using.

Requires the SoapyHoudiniSDR host examples (for `houdini_setup`). Point
HOUDINI_EXAMPLES at them if they are not at the default location, matching how
`tests/hil/beacon_tdd.py` resolves the same dependency.
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SOUNDER = os.path.dirname(_HERE)

# The framer teardown sequence itself lives in the HIL helper. Import it rather
# than copying the register writes: a second copy would silently rot the first
# time the sequence changes.
_HIL = os.path.join(_SOUNDER, "tests", "hil")

# Cross-repo dependency: houdini_setup ships with the SoapyHoudiniSDR host
# examples. Same env var and default that tests/hil uses.
_EXAMPLES = os.environ.get(
    "HOUDINI_EXAMPLES",
    os.path.expanduser("~/repos/SoapyHoudiniSDR/host/examples"))

DEFAULT_TOPOLOGY = os.path.join(_SOUNDER, "files", "topology-houdini.json")
SOAPY_SDR_RX = None   # bound in _import_deps once SoapySDR is importable


def _import_deps():
    """Import the two dependencies, with a message that says how to fix a miss."""
    for p in (_HIL, _EXAMPLES):
        if p not in sys.path:
            sys.path.insert(0, p)
    try:
        import houdini_setup  # noqa: F401
    except ImportError as e:
        sys.stderr.write(
            "error: cannot import houdini_setup (%s)\n"
            "  Looked in: %s\n"
            "  This ships with the SoapyHoudiniSDR host examples. Set\n"
            "  HOUDINI_EXAMPLES to that directory, or check the repo out\n"
            "  alongside this one.\n" % (e, _EXAMPLES))
        return None, None
    try:
        from beacon_tdd import _teardown
    except ImportError as e:
        sys.stderr.write(
            "error: cannot import the teardown helper from beacon_tdd (%s)\n"
            "  Looked in: %s\n" % (e, _HIL))
        return None, None
    import houdini_setup as hs
    global SOAPY_SDR_RX
    import SoapySDR
    SOAPY_SDR_RX = SoapySDR.SOAPY_SDR_RX
    return hs, _teardown


def nodes_from_topology(path):
    """Collect every radio address in a topology file, base stations first.

    Tolerant of both shapes seen in the shipped files: a "Clients" block that is
    a dict with an "sdr" list, or a bare list.
    """
    with open(path, encoding="utf-8") as f:
        topo = json.load(f)
    found = []

    def _add(v):
        if isinstance(v, str):
            found.append(v)
        elif isinstance(v, list):
            found.extend(x for x in v if isinstance(x, str))

    for bs in (topo.get("BaseStations") or {}).values():
        if isinstance(bs, dict):
            _add(bs.get("sdr"))
    clients = topo.get("Clients")
    if isinstance(clients, dict):
        _add(clients.get("sdr"))
    else:
        _add(clients)

    # Preserve order, drop duplicates (a single-board bench lists one address twice).
    seen, ordered = set(), []
    for ip in found:
        if ip not in seen:
            seen.add(ip)
            ordered.append(ip)
    return ordered


def teardown_node(hs, teardown, ip, ch, passes):
    """Tear down one radio. Returns True on success.

    `_teardown` swallows its own exceptions by design, so a failed write shows up
    here only as an exception escaping the open or the settings read. Repeating
    the sequence is the cheap guard against a partially applied first pass; the
    operations are idempotent, so an extra pass costs nothing on a clean board.
    """
    try:
        sdr = hs.open_device(node=ip, ch=ch, verbose=False)["sdr"]
    except Exception as e:  # noqa: BLE001
        print("  %s: FAILED to open (%s: %s)" % (ip, type(e).__name__, str(e)[:120]))
        return False

    try:
        for _ in range(passes):
            teardown(sdr)
    except Exception as e:  # noqa: BLE001
        print("  %s: FAILED during teardown (%s: %s)"
              % (ip, type(e).__name__, str(e)[:120]))
        return False

    # Best effort evidence for the operator. Not a pass/fail gate: the meaning of
    # the state string is the driver's to define, and we do not want a teardown
    # that worked to be reported as failed because the readback surface moved.
    state = ""
    try:
        state = " (TDD_ARM: %s)" % sdr.readSetting("TDD_ARM").strip()
    except Exception:  # noqa: BLE001
        pass

    # This teardown clears the framer, the transmit RAM and the gate. It CANNOT
    # close an RX stream a dead process left open on the device -- and that
    # leftover stream is what makes the next run fail, several layers away, with
    # "setSampleRate(RX): an RX stream is open; a rate change is NOT live".
    # Probe for it here so the operator learns it from the teardown instead of
    # from a confusing failure later. The probe re-applies the CURRENT rate, so
    # it changes nothing when no stream is open.
    try:
        sdr.setSampleRate(SOAPY_SDR_RX, ch, sdr.getSampleRate(SOAPY_SDR_RX, ch))
    except Exception as e:  # noqa: BLE001
        if "stream is open" in str(e):
            # THE HOLDER IS USUALLY A LOCAL PROCESS, SO SAY THAT FIRST.
            # Restarting the server was the only advice here, and it needs a
            # sudo password this bench does not grant. In every case seen on
            # 2026-09-02 the stream was held by an orphaned sounder on THIS
            # host: csi_server.py --launch runs the sounder as a grandchild
            # under a bash retry loop, so killing the server orphans the loop,
            # the loop restarts a sounder, and that sounder holds both boards.
            # Killing it releases the stream immediately -- no server restart.
            # Six consecutive runs were refused that way before anyone noticed,
            # because a refused run writes an EMPTY log rather than an error.
            print("  %s: WARNING an RX stream is still open on the device.%s\n"
                  "      This teardown cannot close another process's stream, and "
                  "the next run will fail\n"
                  "      with 'setSampleRate(RX): an RX stream is open'.\n"
                  "      USUALLY A LOCAL PROCESS IS HOLDING IT -- an orphaned "
                  "sounder left by a run\n"
                  "      whose launcher was killed. Try this first, no sudo "
                  "needed:\n"
                  "          python3 tools/rig_release_holders.py\n"
                  "      Only if that finds nothing does the server itself need "
                  "restarting:\n"
                  "          sudo systemctl restart SoapySDRServer   (on %s)"
                  % (ip, state, ip))
            return False
        # Anything else is not this condition; do not fail the teardown over it.

    print("  %s: torn down%s" % (ip, state))
    return True


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topology", default=DEFAULT_TOPOLOGY,
                    help="topology JSON to read radio addresses from "
                         "(default: %(default)s)")
    ap.add_argument("--node", action="append", metavar="ADDR",
                    help="radio address; repeatable. Overrides --topology.")
    ap.add_argument("--ch", type=int, default=1,
                    help="channel to open (default: %(default)s)")
    ap.add_argument("--passes", type=int, default=2,
                    help="teardown repeats per radio (default: %(default)s)")
    args = ap.parse_args()

    if args.node:
        nodes = args.node
    else:
        try:
            nodes = nodes_from_topology(args.topology)
        except FileNotFoundError:
            sys.stderr.write("error: no topology file at %s\n"
                             "  Pass --topology, or name radios with --node.\n"
                             % args.topology)
            return 2
        except (ValueError, KeyError, TypeError) as e:
            sys.stderr.write("error: cannot read radios from %s (%s)\n"
                             % (args.topology, e))
            return 2

    if not nodes:
        sys.stderr.write("error: no radio addresses found in %s\n" % args.topology)
        return 2

    hs, teardown = _import_deps()
    if hs is None:
        return 2

    print("[teardown] releasing the framer on %d radio(s): %s"
          % (len(nodes), ", ".join(nodes)))
    failed = [ip for ip in nodes
              if not teardown_node(hs, teardown, ip, args.ch, args.passes)]

    # Device handles are released when this process exits, which is immediately
    # after this returns. Holding one open would block the sounder's own open.
    if failed:
        print("[teardown] FAILED on %d of %d: %s"
              % (len(failed), len(nodes), ", ".join(failed)))
        return 1
    print("[teardown] all %d radio(s) clear" % len(nodes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
