#!/usr/bin/env python
"""Nero 专用遥操作脚本.

数据流:
    Xbox 手柄摇杆/按键
        │
        ▼
    NeroGamepad.get_action()
        → ctrl.get_deltas()         → (delta_x, delta_y, delta_z, delta_wx, delta_wy, delta_wz)
        → ctrl.gripper_command()    → delta_gripper
        │
        ▼
    teleop.solve_ik_from_deltas()   → 笛卡尔增量累加到末端位姿 → TracIK → 关节角
                                     (含 max_joint_step 裁剪 + reject_threshold + EMA)
        │
        ▼
    拼装 RobotAction: {"joint1.pos": ..., "joint7.pos": ..., "gripper.pos": ...}
        │
        ▼
    robot.send_action(action)       → Nero.send_action() → move_j/move_js

按键:
    L-stick  → 末端 X/Y 平移
    R-stick  → 末端 Z 平移 / Z 旋转
    D-pad    → 末端 X/Y 旋转
    A        → 夹爪闭合
    B        → 夹爪张开
    Y        → 回 Home 位置 (暂停遥操作, 回 Home 后恢复)
    Back     → E-STOP (回 Home 后断开)
    Home     → 连接机械臂

Usage:
    uv run python scripts/lerobot_teleoperate_nero.py \
        --robot.type=nero --teleop.type=nero_gamepad \
        --display_data=true

    # With Orbbec camera + live display:
    uv run python scripts/lerobot_teleoperate_nero.py \
        --robot.type=nero --teleop.type=nero_gamepad \
        --use_orbbec_camera=true --display_cameras=true
"""

import logging
import os
import time
from dataclasses import asdict, dataclass
from pprint import pformat

import cv2
import numpy as np
from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.orbbec import OrbbecCameraConfig  # noqa: F401
from lerobot.cameras.configs import ColorMode
from lerobot.configs import parser
from lerobot.robots import Robot, RobotConfig, make_robot_from_config, nero  # noqa: F401
from lerobot.robots.nero import Nero
from lerobot.robots.nero.config_nero import NERO_GRIPPER_MAX_WIDTH_M, NERO_JOINT_NAMES
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    make_teleoperator_from_config,
    nero_gamepad,
)
from lerobot.teleoperators.nero_gamepad import NeroGamepad
from lerobot.teleoperators.utils import TeleopEvents
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, shutdown_rerun

logger = logging.getLogger(__name__)


@dataclass
class NeroTeleoperateConfig:
    robot: RobotConfig
    teleop: TeleoperatorConfig
    fps: int = 30
    teleop_time_s: float | None = None
    display_data: bool = False
    display_cameras: bool = False
    use_orbbec_camera: bool = False
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False


def _init_can():
    os.system("sudo ip link set can0 up type can bitrate 1000000 2>/dev/null")
    logger.info("CAN bus initialized (can0, 1Mbps)")


def _sync_ik_from_robot(teleop: NeroGamepad, robot: Nero, retries: int = 10) -> None:
    if teleop.ik_solver is None:
        return

    actual_joints = None
    for attempt in range(retries):
        obs = robot.get_observation()
        if all(f"{name}.pos" in obs for name in NERO_JOINT_NAMES):
            actual_joints = [obs[f"{name}.pos"] for name in NERO_JOINT_NAMES]
            break
        logger.warning(f"get_observation missing joint data (attempt {attempt + 1}/{retries}), retrying...")
        time.sleep(0.5)

    if actual_joints is None:
        logger.warning("Failed to read joint angles from robot, falling back to home")
        actual_joints = list(teleop.config.home_joint_angles)

    teleop.set_joint_angles(actual_joints)
    teleop.ik_solver.set_seed(actual_joints)
    logger.info(f"IK synced to joints: {[f'{v:.3f}' for v in actual_joints]}")


def nero_teleop_loop(
    teleop: NeroGamepad,
    robot: Nero,
    fps: int,
    display_data: bool = False,
    display_cameras: bool = False,
    duration: float | None = None,
    display_compressed_images: bool = False,
):
    if teleop.ik_solver is None or not teleop._ik_initialized:
        raise RuntimeError(
            "IK not initialized — cannot convert Cartesian deltas to joint angles. "
            "Check URDF path and trac_ik installation."
        )

    display_len = max(len(key) for key in robot.action_features)
    start = time.perf_counter()
    going_home = False

    while True:
        loop_start = time.perf_counter()

        raw_action = teleop.get_action()
        events = teleop.get_teleop_events()

        if events.get(TeleopEvents.SUCCESS):
            logger.info("Y pressed — moving to Home position")
            going_home = True
            robot.move_to_home()
            _sync_ik_from_robot(teleop, robot)
            teleop.gripper_state = 100.0
            logger.info(f"Home done. IK joints: {[f'{v:.4f}' for v in teleop.get_joint_angles()]}")
            logger.info(f"Home done. Config home: {[f'{v:.4f}' for v in teleop.config.home_joint_angles]}")
            going_home = False
            continue

        if events.get(TeleopEvents.TERMINATE_EPISODE):
            logger.info("Back pressed — E-STOP, moving to Home then disconnecting")
            robot.move_to_home()
            return

        if teleop.is_home_requested():
            logger.info("Home button pressed — reconnecting robot")
            if not robot.is_connected:
                robot.connect()
                _sync_ik_from_robot(teleop, robot)

        if going_home:
            dt_s = time.perf_counter() - loop_start
            precise_sleep(max(1 / fps - dt_s, 0.0))
            continue

        delta_x = raw_action["delta_x"]
        delta_y = raw_action["delta_y"]
        delta_z = raw_action["delta_z"]
        delta_wx = raw_action["delta_wx"]
        delta_wy = raw_action["delta_wy"]
        delta_wz = raw_action["delta_wz"]
        delta_gripper = raw_action.get("delta_gripper", 0.0)

        has_input = any(v != 0.0 for v in (delta_x, delta_y, delta_z, delta_wx, delta_wy, delta_wz))

        if has_input:
            teleop.solve_ik_from_deltas(delta_x, delta_y, delta_z, delta_wx, delta_wy, delta_wz)

        if delta_gripper != 0.0:
            teleop.update_gripper_state(delta_gripper)

        action = {}
        for name, val in zip(NERO_JOINT_NAMES, teleop.get_joint_angles(), strict=True):
            action[f"{name}.pos"] = val

        if teleop.config.use_gripper:
            action["gripper.pos"] = NERO_GRIPPER_MAX_WIDTH_M * teleop.gripper_state * 1e-2

        robot.send_action(action)

        if display_cameras and robot.cameras:
            for cam_name, cam in robot.cameras.items():
                try:
                    frame = cam.read_latest(max_age_ms=2000)
                    if frame is not None:
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        cv2.imshow(f"Nero - {cam_name}", frame_bgr)
                except (TimeoutError, RuntimeError):
                    pass
            if cv2.waitKey(1) & 0xFF == ord("q"):
                logger.info("Quit from camera window")
                return

        if display_data:
            robot.get_observation()
            print("\n" + "-" * (display_len + 10))
            print(f"{'NAME':<{display_len}} | {'VALUE':>9}")
            for motor, value in action.items():
                print(f"{motor:<{display_len}} | {value:>9.4f}")
            move_cursor_up(len(action) + 3)

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(1 / fps - dt_s, 0.0))
        loop_s = time.perf_counter() - loop_start
        print(f"Teleop loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")
        move_cursor_up(1)

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def teleoperate(cfg: NeroTeleoperateConfig):
    init_logging()

    if cfg.use_orbbec_camera:
        cfg.robot.cameras["top"] = OrbbecCameraConfig(
            fps=30, width=1280, height=720,
            color_mode=ColorMode.RGB, warmup_s=2,
            auto_exposure=True, auto_white_balance=True,
        )
        logger.info("Orbbec camera attached as 'top'")

    logging.info(pformat(asdict(cfg)))

    _init_can()

    if cfg.display_data:
        init_rerun(session_name="nero_teleoperation", ip=cfg.display_ip, port=cfg.display_port)
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)

    teleop.connect()
    robot.connect()

    _sync_ik_from_robot(teleop, robot)

    try:
        nero_teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            display_cameras=cfg.display_cameras,
            duration=cfg.teleop_time_s,
            display_compressed_images=display_compressed_images,
        )
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_cameras:
            cv2.destroyAllWindows()
        if cfg.display_data:
            shutdown_rerun()
        teleop.disconnect()
        robot.disconnect()


def main():
    teleoperate()


if __name__ == "__main__":
    main()
