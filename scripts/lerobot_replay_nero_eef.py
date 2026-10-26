"""Nero 数据集回放 — 支持 EEF pose 数据集 (--eef_pose_state=true 采集) 和关节数据集.

EEF 数据集不能用官方 lerobot-replay (它按 action 名直接映射 robot action features,
没有 eef.* 名字). 本脚本对 EEF action 做与采集时互逆的转换:
    action (末端位姿) → IK(seed=当前实测关节角, 单帧限幅) → move_js 下发

流程:
    连接 → go home → (EEF 模式: move_j 预移到首帧位姿) → 按数据集 fps 逐帧回放

Usage:
    python scripts/lerobot_replay_nero_eef.py \
        --repo_id=eef_task_20261022_170000 \
        --root=/home/zgj/nero_data/dataset/20261022_170000/eef_task_20261022_170000 \
        --episode=0 --fps=30
"""

import os

os.environ.setdefault("ORBBEC_SDK_DISABLE_UVC", "1")

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nero_eef_utils import create_eef_ik_adapter, eef_dict_to_joints

NERO_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


def read_current_joints(robot):
    obs = robot.get_observation()
    joints = [obs.get(f"{name}.pos") for name in NERO_JOINT_NAMES]
    if any(v is None for v in joints):
        return None
    return [float(v) for v in joints]


def pre_move_to_joints(robot, joints):
    """move_j (插补) 安全预移到指定关节角, 速度用 home_speed_percent, 然后切回 JS 伺服模式."""
    robot.arm.set_speed_percent(robot.config.home_speed_percent)
    robot.arm.set_motion_mode(robot.arm.OPTIONS.MOTION_MODE.J)
    robot.arm.move_j(list(joints))
    robot._wait_joints_at_target(list(joints), timeout=30.0)
    robot.arm.set_motion_mode(robot.arm.OPTIONS.MOTION_MODE.JS)


def main():
    parser = argparse.ArgumentParser(description="Nero dataset replay (EEF / joints)")
    parser.add_argument("--repo_id", type=str, required=True)
    parser.add_argument("--root", type=str, default=None)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--fps", type=int, default=None, help="默认用数据集 fps")
    parser.add_argument("--max_joint_step", type=float, default=0.15, help="EEF 模式单帧关节限幅 (rad)")
    parser.add_argument("--home_speed_percent", type=int, default=10, help="回 home / 预移的 move_j 速度百分比")
    args = parser.parse_args()

    from lerobot.datasets import LeRobotDataset
    from lerobot.robots.nero import Nero, NeroRobotConfig
    from lerobot.utils.constants import ACTION
    from lerobot.utils.robot_utils import precise_sleep

    dataset = LeRobotDataset(args.repo_id, root=args.root, episodes=[args.episode])
    fps = args.fps or dataset.fps
    action_names = list(dataset.features[ACTION]["names"])
    eef_mode = "eef.x" in action_names
    print(f"Dataset: {args.repo_id} ep{args.episode}, {dataset.num_frames} frames @ {fps}fps, mode={'eef' if eef_mode else 'joints'}")

    actions = dataset.select_columns(ACTION)

    ik_adapter = None
    if eef_mode:
        ik_adapter = create_eef_ik_adapter()
        if ik_adapter is None:
            raise RuntimeError("EEF dataset detected but TracIK init failed (check URDF)")

    robot = Nero(NeroRobotConfig(motion_mode="js", use_gripper=True, home_speed_percent=args.home_speed_percent))
    robot.connect()
    robot.move_to_home()

    try:
        if eef_mode:
            # 预移到首帧目标位姿, 保证回放起点与录制起点一致
            seed = read_current_joints(robot)
            first = {n: float(v) for n, v in zip(action_names, actions[0][ACTION])}
            start_joints = eef_dict_to_joints(ik_adapter, first, seed, max_joint_step=0)
            if start_joints is None:
                raise RuntimeError("IK failed for first frame of episode")
            print(f"Pre-moving to episode start pose: {[f'{v:.3f}' for v in start_joints]}")
            pre_move_to_joints(robot, start_joints)
            gripper_open = {"gripper.pos": first.get("gripper.pos", 0.0)}

        for idx in range(dataset.num_frames):
            t0 = time.perf_counter()

            row = actions[idx][ACTION]
            values = {n: float(v) for n, v in zip(action_names, row)}

            if eef_mode:
                seed = read_current_joints(robot) or list(start_joints)
                joints = eef_dict_to_joints(ik_adapter, values, seed, args.max_joint_step)
                if joints is None:
                    joints = seed  # IK 失败: 保持当前关节角
                action_dict = {f"{name}.pos": j for name, j in zip(NERO_JOINT_NAMES, joints)}
                action_dict["gripper.pos"] = values.get("gripper.pos", 0.0)
            else:
                action_dict = {k: v for k, v in values.items() if k.endswith(".pos")}

            robot.send_action(action_dict)

            dt_s = time.perf_counter() - t0
            precise_sleep(max(1 / fps - dt_s, 0.0))

            if idx % 60 == 0:
                print(f"  frame {idx}/{dataset.num_frames}")

        print("Replay done.")
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
