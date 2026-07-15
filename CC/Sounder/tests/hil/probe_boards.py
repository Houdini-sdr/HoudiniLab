#!/usr/bin/env python3
"""
probe_boards.py -- inventory the Houdini RFSoC4x2 boards for the Sounder HIL
bring-up. Opens each board by IP and prints its RX/TX channel map
(getChannelInfo incl. the rfdc_label, plus listAntennas), so we can read off the
SoapySDR channel index for a given SMA converter -- the map is *dynamic*
(BuildChannelMaps assigns sequential indices over the *enabled* converters), so
we discover it rather than assume "ADC_C == ch 2".

Rig: DAC_A on .21 (TX) -> ADC_C on .22 (RX), both cabled to the DGX Spark (100G).
Self-contained (only SoapySDR); confirms both boards are reachable + gives the
channel indices the tone / beacon scripts need.

Run on the DGX (after: source houdini_test/bin/activate):
    python3 probe_boards.py 168.6.244.21 168.6.244.22
"""
import sys

import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX


def find_by_ip(ip, results):
    """Pick the discovery result whose kwargs mention this IP."""
    for r in results:
        d = dict(r)
        if ip in str(d) or any(ip in str(v) for v in d.values()):
            return r
    return None


def dump_channels(sdr, direction, tag):
    try:
        n = sdr.getNumChannels(direction)
    except Exception as e:  # noqa: BLE001
        print(f"  {tag}: getNumChannels failed: {e}")
        return
    print(f"  {tag} channels: {n}")
    for ch in range(n):
        try:
            info = dict(sdr.getChannelInfo(direction, ch))
        except Exception as e:  # noqa: BLE001
            info = {"error": str(e)}
        label = info.get("rfdc_label") or info.get("label") or "?"
        try:
            ants = list(sdr.listAntennas(direction, ch))
        except Exception:  # noqa: BLE001
            ants = []
        try:
            fs = sdr.getSampleRate(direction, ch) / 1e6
        except Exception:  # noqa: BLE001
            fs = float("nan")
        try:
            fc = sdr.getFrequency(direction, ch) / 1e6
        except Exception:  # noqa: BLE001
            fc = float("nan")
        print(f"    {tag} ch{ch}: label={label}  antennas={ants}  "
              f"fs={fs:.3f} MSPS  nco/freq={fc:.3f} MHz")
        extra = {k: v for k, v in info.items()
                 if k not in ("rfdc_label", "label")}
        if extra:
            print(f"            info={extra}")


def main():
    ips = sys.argv[1:] or ["168.6.244.21", "168.6.244.22"]
    print("Enumerating houdinisdr devices ...")
    results = list(SoapySDR.Device.enumerate(dict(driver="houdinisdr")))
    print(f"  discovered {len(results)} device(s):")
    for r in results:
        print("   ", dict(r))
    print()

    for ip in ips:
        print(f"===== board {ip} =====")
        sel = find_by_ip(ip, results)
        if sel is None:
            print(f"  NOT FOUND in discovery -- is {ip} up + reachable from the DGX?")
            print()
            continue
        try:
            sdr = SoapySDR.Device(sel)
        except Exception as e:  # noqa: BLE001
            print(f"  open FAILED: {e}")
            print()
            continue
        try:
            print(f"  hardware: key={sdr.getHardwareKey()}  "
                  f"info={dict(sdr.getHardwareInfo())}")
        except Exception as e:  # noqa: BLE001
            print(f"  getHardwareInfo failed: {e}")
        dump_channels(sdr, SOAPY_SDR_TX, "TX")
        dump_channels(sdr, SOAPY_SDR_RX, "RX")
        print()

    print("Read off: DAC_A -> a TX channel index on .21 ; ADC_C -> an RX channel "
          "index on .22 (match by label/antenna).")


if __name__ == "__main__":
    main()
