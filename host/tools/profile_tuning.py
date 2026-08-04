#!/usr/bin/env python3
"""Compare ST ProfileTuning estimates with the roomscanner profile model."""

from __future__ import annotations

import argparse
import json
from roomscan.profiles import (
    AMBIENT_LUX_DEFAULT,
    ManualParams,
    PowerMode,
    RangingMode,
    estimate_manual,
    estimate_to_json,
)

# ST's own per-resolution frame sizes, from the decompiled tool. Note 54x42 is
# 14,742 = 2268 zones x 6.5 B exactly, while our wire payload
# (`RAW_3DMD_BYTES_BIN2`, docs/protocol.md) is 14,842 -- 100 B more. That is an
# accounting difference between the two models, NOT a typo in either: do not
# "reconcile" them by editing one, or the 0.08 ms readout gap disappears while
# the disagreement it represents does not.
_ST_FRAME_BYTES_DSS_ON = {"54x42": 14742.0, "24x20": 3744.0, "18x14": 1638.0,
                          "12x10": 780.0, "8x6": 416.0, "4x4": 104.0}
_ST_DSS_MS = {"54x42": 13.5, "24x20": 6.75, "18x14": 2.25,
              "12x10": 1.6875, "8x6": 1.125, "4x4": 0.5625}
_INTERFACE_BPS = {"CSI2": 1e9, "I3C": 10e6, "I2C_FM+": 1e6, "I2C_FM": 400e3}
_DEAD_TIME_MS = {"Regular": 1.5, "Low Power": 6.0, "Ultralow Power": 8.0}


def st_profiletuning_estimate(*, resolution: str, power_config: str, dss: bool,
                              output_interface: str, exposure_ms: float) -> dict:
    """Return the decompiled ST planning calculation as a named comparison source."""
    frame_bytes = _ST_FRAME_BYTES_DSS_ON[resolution] if dss else 106.0
    readout_ms = frame_bytes * 8 * 1000 / _INTERFACE_BPS[output_interface]
    active_ms = max(exposure_ms * 1.15 + 2.0, _ST_DSS_MS[resolution] if dss else 0.0)
    frame_ms = _DEAD_TIME_MS[power_config] + readout_ms + active_ms
    return {"source": "ST ProfileTuning.exe decompiled model", "frame_bytes": frame_bytes,
            "readout_ms": round(readout_ms, 4), "sensor_active_ms": round(active_ms, 4),
            "max_fps": round(1000.0 / frame_ms, 2),
            "note": "Displayed duty cycle is sensor active duty, not I3C bus duty."}


def analyze_profile(*, ranging_mode: str = "Precision", power_config: str = "Regular",
                    resolution: str = "54x42", dss: bool = True,
                    output_interface: str = "I3C", fps: int = 30,
                    exposure_ms: int = 10,
                    ambient_lux: float = AMBIENT_LUX_DEFAULT) -> dict:
    """Compare ST's planning model with the rig's full-map hardware-backed model.

    `ambient_lux` is a real input to ST's power equations (the Vbat Rx branch
    scales with it), so it reaches `roomscanner_full_map.power_mw`; ST's own
    timing model does not use it, which is why only one side of the comparison
    moves when you change it.
    """
    mode = RangingMode.PRECISION if ranging_mode == "Precision" else RangingMode.AMBIENT
    power = {"Regular": PowerMode.REGULAR, "Low Power": PowerMode.LOW,
             "Ultralow Power": PowerMode.ULTRA_LOW}[power_config]
    canonical = estimate_manual(ManualParams(mode, fps, exposure_ms, power),
                                ambient_lux=ambient_lux)
    st = st_profiletuning_estimate(resolution=resolution, power_config=power_config,
                                   dss=dss, output_interface=output_interface,
                                   exposure_ms=exposure_ms)
    disagreements = list(canonical.warnings) + list(canonical.errors)
    if output_interface != "I3C":
        disagreements.append(
            "Interface scope conflict: roomscanner_full_map is an I3C-only model; "
            "CSI-2 and I2C estimates are ProfileTuning planning estimates, not rig predictions.")
    if canonical.dss_enabled != dss:
        disagreements.append(
            f"DSS policy conflict: the request says DSS={'on' if dss else 'off'}, but "
            f"the roomscanner profile policy derives DSS={'on' if canonical.dss_enabled else 'off'} "
            f"at {fps} fps.")
    if output_interface == "I3C" and resolution == "54x42" and not dss:
        disagreements.append(
            "DSS-off I3C conflict: ProfileTuning assumes 106 bytes, while the verified "
            "roomscanner full-map pipeline reserves 14,842 bytes. The vendor I3C reader "
            "always fetches the DSS LUT; DSS-off full-map I3C is unverified.")
    if output_interface == "I3C" and resolution == "54x42" and abs(st["max_fps"] - canonical.expected_delivered_fps) > 1.0:
        disagreements.append(
            f"Rate conflict: ST planning model says max {st['max_fps']:.1f} fps; "
            f"hardware-backed model expects {canonical.expected_delivered_fps:.1f} fps "
            f"for the {fps} fps request.")
    return {"request": {"ranging_mode": ranging_mode, "power_config": power_config,
                        "resolution": resolution, "dss_requested": dss,
                        "output_interface": output_interface, "fps": fps,
                        "exposure_ms": exposure_ms, "ambient_lux": ambient_lux},
            "st_profiletuning": st,
            # `estimate_to_json`, not `asdict`: one spelling of an estimate across
            # the whole surface (`/ws` `ranging`, `profile_estimate`, `rig_profile`)
            # -- asdict emitted raw enum ints here and silently dropped `ok`, which
            # is a property rather than a field.
            "roomscanner_full_map": estimate_to_json(canonical),
            "agree": not disagreements, "disagreements": disagreements}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-r", "--ranging-mode", choices=["Precision", "Ambient"], default="Precision")
    parser.add_argument("-s", "--resolution", choices=list(_ST_FRAME_BYTES_DSS_ON), default="54x42")
    parser.add_argument("-p", "--power-config", choices=list(_DEAD_TIME_MS), default="Regular")
    parser.add_argument("--dss", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("-i", "--output-interface", choices=list(_INTERFACE_BPS), default="I3C")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("-e", "--exposure", dest="exposure_ms", type=int, default=10)
    parser.add_argument("-a", "--ambient-lux", type=float, default=AMBIENT_LUX_DEFAULT,
                        help="ambient light for the power model (ST's Vbat Rx term)")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    report = analyze_profile(**{key: value for key, value in vars(args).items()
                                if key != "json"})
    if args.json:
        print(json.dumps(report, indent=2))
        return
    st = report["st_profiletuning"]
    rig = report["roomscanner_full_map"]
    print("ST ProfileTuning vs roomscanner full-map model")
    print(f"ST planning: {st['max_fps']:.2f} fps max, {st['readout_ms']:.4f} ms readout, "
          f"{st['frame_bytes']:.0f} B")
    print(f"Rig model: expected {rig['expected_delivered_fps']:.2f} fps, "
          f"{rig['i3c_xfer_ms']:.3f} ms I3C readout, "
          f"{rig['i3c_bus_utilization_pct']:.1f}% I3C airtime, "
          f"{rig['power_mw']:.1f} mW at {args.ambient_lux:.0f} lux")
    if report["disagreements"]:
        print("DISAGREEMENTS:")
        for message in report["disagreements"]:
            print(f"- {message}")
    else:
        print("Sources agree within the model's stated scope.")


if __name__ == "__main__":
    main()
