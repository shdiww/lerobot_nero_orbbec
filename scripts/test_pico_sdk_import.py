#!/usr/bin/env python
"""Test 1: Verify xrobotoolkit_sdk can be imported and initialized.

Usage:
    python scripts/test_pico_sdk_import.py

This script only checks that the SDK module is available and can call init().
It does NOT require the PC Service to be running.
"""

import sys


def test_import():
    print("[1/3] Importing xrobotoolkit_sdk ...")
    try:
        import xrobotoolkit_sdk as xrt
        print(f"  OK - module path: {xrt.__file__}")
    except ImportError as e:
        print(f"  FAIL - Cannot import xrobotoolkit_sdk: {e}")
        print("\n  To install, run:")
        print("    cd /home/blzgz/XRoboToolkit-Teleop-Sample-Python/dependencies/XRoboToolkit-PC-Service-Pybind")
        print("    bash setup_ubuntu.sh")
        return False
    return True


def test_api_presence():
    print("[2/3] Checking SDK API functions ...")
    import xrobotoolkit_sdk as xrt

    required = [
        "init", "close",
        "get_right_controller_pose", "get_left_controller_pose", "get_headset_pose",
        "get_right_grip", "get_left_grip", "get_right_trigger", "get_left_trigger",
        "get_A_button", "get_B_button", "get_X_button", "get_Y_button",
        "get_right_axis", "get_left_axis",
        "get_right_axis_click", "get_left_axis_click",
        "get_time_stamp_ns",
    ]
    missing = [fn for fn in required if not hasattr(xrt, fn)]
    if missing:
        print(f"  FAIL - Missing functions: {missing}")
        return False
    print(f"  OK - All {len(required)} API functions present")
    return True


def test_init_close():
    print("[3/3] Testing init() / close() ...")
    import xrobotoolkit_sdk as xrt
    try:
        xrt.init()
        print("  OK - init() succeeded")
    except Exception as e:
        print(f"  FAIL - init() raised: {e}")
        print("  (This may be normal if PC Service is not running)")
        return False
    try:
        xrt.close()
        print("  OK - close() succeeded")
    except Exception as e:
        print(f"  WARN - close() raised: {e}")
    return True


def main():
    print("=" * 60)
    print("Pico VR SDK Import Test")
    print("=" * 60)

    results = []
    results.append(("Import", test_import()))
    if results[-1][1]:
        results.append(("API", test_api_presence()))
        results.append(("Init/Close", test_init_close()))

    print("\n" + "=" * 60)
    print("Results:")
    for name, ok in results:
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {name}")
    print("=" * 60)

    all_ok = all(ok for _, ok in results)
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
