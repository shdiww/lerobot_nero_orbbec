#!/usr/bin/env python
"""Test 2: Verify Pico VR controller data can be read.

Usage:
    python scripts/test_pico_data_read.py [--duration 10]

Prerequisites:
    - xrobotoolkit_sdk installed (run test_pico_sdk_import.py first)
    - XRoboToolkit PC Service running on the same machine
    - Pico VR headset powered on and connected to same WiFi

This script reads and prints controller data for the specified duration.
"""

import argparse
import time
import sys


R_HEADSET_TO_WORLD = [
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
]


def test_read_data(duration_s: int = 10):
    import xrobotoolkit_sdk as xrt
    import numpy as np

    print("Initializing SDK ...")
    try:
        xrt.init()
    except Exception as e:
        print(f"FAIL - xrt.init() raised: {e}")
        print("Make sure XRoboToolkit PC Service is running!")
        return False

    R = np.array(R_HEADSET_TO_WORLD, dtype=np.float64)
    start = time.monotonic()
    frame = 0
    got_pose = False
    got_button = False
    got_axis = False
    got_grip = False

    print(f"\nReading Pico data for {duration_s}s ... (Ctrl+C to stop early)\n")

    try:
        while time.monotonic() - start < duration_s:
            loop_start = time.monotonic()

            # --- Right controller pose ---
            try:
                pose = xrt.get_right_controller_pose()
                if pose is not None and len(pose) == 7:
                    xyz = np.array(pose[:3])
                    xyz_world = R @ xyz
                    print(f"  Right pose (world): x={xyz_world[0]:.4f} y={xyz_world[1]:.4f} z={xyz_world[2]:.4f} | "
                          f"qx={pose[3]:.3f} qy={pose[4]:.3f} qz={pose[5]:.3f} qw={pose[6]:.3f}")
                    got_pose = True
                else:
                    print("  Right pose: <no data / wrong format>")
            except Exception as e:
                print(f"  Right pose: ERROR - {e}")

            # --- Grip & trigger ---
            try:
                grip = xrt.get_right_grip()
                trigger = xrt.get_right_trigger()
                print(f"  Grip={grip:.3f}  Trigger={trigger:.3f}")
                got_grip = True
            except Exception as e:
                print(f"  Grip/Trigger: ERROR - {e}")

            # --- Buttons ---
            try:
                a_btn = xrt.get_A_button()
                b_btn = xrt.get_B_button()
                x_btn = xrt.get_X_button()
                y_btn = xrt.get_Y_button()
                print(f"  A={a_btn} B={b_btn} X={x_btn} Y={y_btn}")
                got_button = True
            except Exception as e:
                print(f"  Buttons: ERROR - {e}")

            # --- Joystick ---
            try:
                axis = xrt.get_right_axis()
                print(f"  Joystick: x={axis[0]:.3f} y={axis[1]:.3f}")
                got_axis = True
            except Exception as e:
                print(f"  Joystick: ERROR - {e}")

            frame += 1
            loop_ms = (time.monotonic() - loop_start) * 1000
            print(f"  --- frame {frame}, loop={loop_ms:.1f}ms ---\n")

            time.sleep(max(0.05 - loop_ms / 1000, 0))

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    xrt.close()

    print("=" * 60)
    print("Data Read Results:")
    print(f"  Frames read:   {frame}")
    print(f"  Got pose:      {'YES' if got_pose else 'NO'}")
    print(f"  Got grip:      {'YES' if got_grip else 'NO'}")
    print(f"  Got buttons:   {'YES' if got_button else 'NO'}")
    print(f"  Got joystick:  {'YES' if got_axis else 'NO'}")

    if not got_pose:
        print("\n  WARNING: No pose data received.")
        print("  - Is the Pico headset powered on and on the same WiFi?")
        print("  - Is XRoboToolkit PC Service running?")
        return False

    return True


def main():
    parser = argparse.ArgumentParser(description="Test Pico VR data reading")
    parser.add_argument("--duration", type=int, default=10, help="Test duration in seconds")
    args = parser.parse_args()

    print("=" * 60)
    print("Pico VR Data Read Test")
    print("=" * 60)

    ok = test_read_data(args.duration)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
