#!/usr/bin/env python
"""Test 3: Verify Pico VR 6D tracking deltas with IK solving.

Usage:
    python scripts/test_pico_vr_tracking.py [--duration 30]

Prerequisites:
    - xrobotoolkit_sdk installed
    - XRoboToolkit PC Service running
    - Pico VR headset powered on and connected

This script creates a NeroPicoVR teleoperator and prints:
  - Raw 6D deltas from Pico controller
  - IK-solved joint angles (if URDF found)

No real robot needed. Grip the right controller to activate tracking.
"""

import argparse
import signal
import sys
import time


def run_test(duration_s: int = 30):
    from lerobot.teleoperators.pico_vr import NeroPicoVR, NeroPicoVRConfig

    cfg = NeroPicoVRConfig()
    teleop = NeroPicoVR(cfg)

    print("Connecting to Pico VR ...")
    try:
        teleop.connect()
    except Exception as e:
        print(f"FAIL - connect() raised: {e}")
        return False

    print(f"Connected: {teleop.is_connected}")
    print(f"IK solver: {'YES' if teleop.ik_solver else 'NO (deltas only)'}")

    if teleop.ik_solver:
        print(f"Home joints: {[f'{v:.3f}' for v in teleop.get_joint_angles()]}")

    print(f"\nReading for {duration_s}s ... Grip the RIGHT controller to activate tracking.")
    print("=" * 80)

    start = time.monotonic()
    frame = 0
    stop = False

    def _signal_handler(sig, frame_arg):
        nonlocal stop
        stop = True
        print("\nCtrl+C caught, stopping...")

    signal.signal(signal.SIGINT, _signal_handler)

    while time.monotonic() - start < duration_s and not stop:
        loop_start = time.monotonic()

        action = teleop.get_action()

        dx = action["delta_x"]
        dy = action["delta_y"]
        dz = action["delta_z"]
        dwx = action["delta_wx"]
        dwy = action["delta_wy"]
        dwz = action["delta_wz"]
        dg = action.get("delta_gripper", 0.0)

        has_input = any(v != 0.0 for v in (dx, dy, dz, dwx, dwy, dwz))

        if has_input and teleop.ik_solver:
            teleop.solve_ik_from_deltas(dx, dy, dz, dwx, dwy, dwz)

        if dg != 0.0:
            teleop.update_gripper_state(dg)

        pos_str = f"dx={dx:+.4f} dy={dy:+.4f} dz={dz:+.4f}"
        rot_str = f"dwx={dwx:+.4f} dwy={dwy:+.4f} dwz={dwz:+.4f}"
        grip_str = f"grip_delta={dg:+.1f} grip_state={teleop.gripper_state:.0f}"

        if teleop.ik_solver:
            joints = teleop.get_joint_angles()
            joint_str = " ".join(f"{v:+.3f}" for v in joints)
            print(f"F{frame:04d} | {pos_str} | {rot_str} | {grip_str} | J=[{joint_str}]")
        else:
            print(f"F{frame:04d} | {pos_str} | {rot_str} | {grip_str}")

        frame += 1
        dt = time.monotonic() - loop_start
        time.sleep(max(1 / 30 - dt, 0))

    teleop.disconnect()
    print("=" * 80)
    print(f"Done. {frame} frames read.")
    return True


def main():
    parser = argparse.ArgumentParser(description="Test Pico VR 6D tracking + IK")
    parser.add_argument("--duration", type=int, default=30, help="Test duration in seconds")
    args = parser.parse_args()

    print("=" * 80)
    print("Pico VR 6D Tracking + IK Test")
    print("=" * 80)

    ok = run_test(args.duration)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
