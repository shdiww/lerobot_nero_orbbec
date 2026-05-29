#!/usr/bin/env python
"""Test 4: Visualize Pico VR 6D tracking deltas as curves.

Usage:
    python scripts/test_pico_vr_plot.py [--duration 15]

Record Pico VR data for the specified duration, then plot 6D deltas + joints.
"""

import argparse
import signal
import sys
import time

import numpy as np


def run_and_plot(duration_s: int = 15):
    import matplotlib.pyplot as plt
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
    print(f"IK solver: {'YES' if teleop.ik_solver else 'NO'}")
    print(f"\nRecording for {duration_s}s ... Grip the RIGHT controller to activate tracking.")
    print("Ctrl+C to stop early.\n")

    records = {
        "time": [],
        "dx": [], "dy": [], "dz": [],
        "dwx": [], "dwy": [], "dwz": [],
        "delta_gripper": [], "gripper_state": [],
        "active": [],
    }
    if teleop.ik_solver:
        for j in range(7):
            records[f"joint{j+1}"] = []

    stop = False

    def _signal_handler(sig, frame_arg):
        nonlocal stop
        stop = True
        print("\nCtrl+C caught, stopping...")

    signal.signal(signal.SIGINT, _signal_handler)

    start = time.monotonic()
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

        t = time.monotonic() - start
        records["time"].append(t)
        records["dx"].append(dx)
        records["dy"].append(dy)
        records["dz"].append(dz)
        records["dwx"].append(dwx)
        records["dwy"].append(dwy)
        records["dwz"].append(dwz)
        records["delta_gripper"].append(dg)
        records["gripper_state"].append(teleop.gripper_state)
        records["active"].append(has_input)

        if teleop.ik_solver:
            for j, val in enumerate(teleop.get_joint_angles()):
                records[f"joint{j+1}"].append(val)

        dt = time.monotonic() - loop_start
        time.sleep(max(1 / 30 - dt, 0))

        elapsed = time.monotonic() - start
        status = "ACTIVE" if has_input else "idle"
        print(f"\r  t={elapsed:.1f}s  {status}  dx={dx:+.4f} dy={dy:+.4f} dz={dz:+.4f}", end="", flush=True)

    teleop.disconnect()
    print("\n\nDisconnected. Generating plots...")

    for key in records:
        records[key] = np.array(records[key])

    t = records["time"]
    n_joints = 7 if teleop.ik_solver else 0

    fig, axes = plt.subplots(4 + (1 if n_joints else 0), 1, figsize=(14, 14), sharex=True)
    fig.suptitle("Pico VR 6D Tracking Test", fontsize=14)

    # 1. Position deltas
    ax = axes[0]
    ax.plot(t, records["dx"], label="dx", linewidth=0.8)
    ax.plot(t, records["dy"], label="dy", linewidth=0.8)
    ax.plot(t, records["dz"], label="dz", linewidth=0.8)
    ax.set_ylabel("Position Delta (m)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    # shade active regions
    active = records["active"].astype(float)
    ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1], where=active > 0, alpha=0.1, color="green", label="grip active")

    # 2. Rotation deltas
    ax = axes[1]
    ax.plot(t, records["dwx"], label="dwx", linewidth=0.8)
    ax.plot(t, records["dwy"], label="dwy", linewidth=0.8)
    ax.plot(t, records["dwz"], label="dwz", linewidth=0.8)
    ax.set_ylabel("Rotation Delta (rad)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1], where=active > 0, alpha=0.1, color="green")

    # 3. Position delta magnitude
    ax = axes[2]
    pos_mag = np.sqrt(records["dx"]**2 + records["dy"]**2 + records["dz"]**2)
    rot_mag = np.sqrt(records["dwx"]**2 + records["dwy"]**2 + records["dwz"]**2)
    ax.plot(t, pos_mag, label="|pos delta|", linewidth=0.8, color="blue")
    ax2 = ax.twinx()
    ax2.plot(t, rot_mag, label="|rot delta|", linewidth=0.8, color="red")
    ax.set_ylabel("|pos delta| (m)", color="blue")
    ax2.set_ylabel("|rot delta| (rad)", color="red")
    ax.legend(loc="upper left", fontsize=8)
    ax2.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4. Gripper
    ax = axes[3]
    ax.plot(t, records["gripper_state"], label="gripper %", linewidth=0.8, color="purple")
    ax.set_ylabel("Gripper State (%)")
    ax.set_ylim(-5, 105)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 5. Joint angles
    if n_joints:
        ax = axes[4]
        for j in range(7):
            ax.plot(t, records[f"joint{j+1}"], label=f"J{j+1}", linewidth=0.8)
        ax.set_ylabel("Joint Angle (rad)")
        ax.legend(loc="upper right", fontsize=8, ncol=4)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()
    return True


def main():
    parser = argparse.ArgumentParser(description="Visualize Pico VR 6D tracking")
    parser.add_argument("--duration", type=int, default=15, help="Recording duration (s)")
    args = parser.parse_args()

    print("=" * 60)
    print("Pico VR 6D Tracking Visualization")
    print("=" * 60)

    ok = run_and_plot(args.duration)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
