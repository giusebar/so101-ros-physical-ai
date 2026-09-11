#!/usr/bin/env python3
"""Validate a LeRobot-style calibration JSON and merge it onto the bundled
per-joint tuning base, writing the Feetech driver's `joints:` YAML schema.

The calibration JSON only carries id/drive_mode/homing_offset/range_min/range_max;
the servo tuning (p/i/d_coefficient, acceleration, gripper torque limits) lives
in the bundled *_joints.yaml and must be preserved, otherwise the driver falls
back to the weak URDF-default gains (p=16/d=32) and the follower can't track.

Usage:
    wrap_calibration.py <input.json> <output.yaml> <role> [tuning_base.yaml]

Exits non-zero on failure (bad JSON, missing joints, missing per-joint id, or an
unreadable tuning base).
"""
import json
import sys

import yaml

EXPECTED_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def main(argv):
    if len(argv) < 4:
        print(
            "usage: wrap_calibration.py <input.json> <output.yaml> <role> "
            "[tuning_base.yaml]",
            file=sys.stderr,
        )
        return 2

    src, dst, role = argv[1], argv[2], argv[3]
    tuning_path = argv[4] if len(argv) > 4 else ""

    try:
        with open(src) as f:
            data = json.load(f)
    except Exception as e:  # noqa: BLE001
        print(
            f"so101-bringup: failed to parse {role} calibration JSON '{src}': {e}",
            file=sys.stderr,
        )
        return 1

    if not isinstance(data, dict) or not data:
        print(
            f"so101-bringup: {role} calibration '{src}' is not a non-empty JSON object",
            file=sys.stderr,
        )
        return 1

    missing = [j for j in EXPECTED_JOINTS if j not in data]
    if missing:
        print(
            f"so101-bringup: {role} calibration '{src}' is missing joints: {missing}",
            file=sys.stderr,
        )
        return 1

    for name, entry in data.items():
        if not isinstance(entry, dict) or "id" not in entry:
            print(
                f"so101-bringup: {role} calibration joint '{name}' in '{src}' has no 'id'",
                file=sys.stderr,
            )
            return 1

    # Load the bundled per-joint tuning base (p/i/d_coefficient, acceleration,
    # gripper torque limits, ...). Calibration values (id/homing_offset/range_*/
    # drive_mode) are overlaid on top so a fresh calibration wins for geometry
    # while the hand-tuned servo gains are preserved.
    tuning = {}
    if tuning_path:
        try:
            with open(tuning_path) as f:
                base = yaml.safe_load(f) or {}
            tuning = base.get("joints", {}) or {}
        except FileNotFoundError:
            print(
                f"so101-bringup: {role} tuning base '{tuning_path}' not found; "
                f"servo gains will fall back to URDF defaults",
                file=sys.stderr,
            )
        except Exception as e:  # noqa: BLE001
            print(
                f"so101-bringup: failed to parse {role} tuning base '{tuning_path}': {e}",
                file=sys.stderr,
            )
            return 1

    merged = {}
    for name, cal in data.items():
        entry = dict(tuning.get(name, {}))  # start from tuning (gains, accel, ...)
        entry.update(cal)                   # calibration overrides id/homing/range
        merged[name] = entry

    with open(dst, "w") as f:
        yaml.safe_dump({"joints": merged}, f, default_flow_style=False, sort_keys=False)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
