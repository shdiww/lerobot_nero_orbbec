#!/usr/bin/env python
"""Nero + Pico VR 纯遥操作脚本 (无录制).

数据流:
    Pico VR 右控制器 6D 位姿
        │
        ▼
    NeroPicoVR.get_action()
        → ctrl.get_deltas()         → (delta_x, delta_y, delta_z, delta_wx, delta_wy, delta_wz)
        → ctrl.gripper_command()    → delta_gripper
        │
        ▼
    teleop.solve_ik_from_deltas()   → 笛卡尔增量累加 → TracIK → 关节角
        │
        ▼
    拼装 RobotAction: → robot.send_action()

Pico VR 按键:
    Grip (>0.9) → 激活 6D 位姿追踪 (握住移动, 松开冻结)
    A           → 夹爪闭合 (增量)
    B           → 夹爪张开 (增量)
    X           → 回 Home 位置
    Y           → 标记成功 / 回 Home
    左 menu     → E-STOP (回 Home 后断开)
    右 menu     → (预留)

Usage:
    uv run python scripts/lerobot_teleoperate_nero_pico.py \
        --robot.type=nero --teleop.type=pico_vr \
        --display_cameras=true
"""

import os

os.environ.setdefault("ORBBEC_SDK_DISABLE_UVC", "1")

import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat

import cv2
import numpy as np
import xrobotoolkit_sdk as xrt
from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.orbbec import OrbbecCameraConfig  # noqa: F401
from lerobot.cameras.realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.cameras.configs import ColorMode
from lerobot.configs import parser
from lerobot.robots import Robot, RobotConfig, make_robot_from_config, nero  # noqa: F401
from lerobot.robots.nero import Nero
from lerobot.robots.nero.config_nero import NERO_GRIPPER_MAX_WIDTH_M, NERO_JOINT_NAMES
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    make_teleoperator_from_config,
    pico_vr,
)
from lerobot.teleoperators.pico_vr import NeroPicoVR
from lerobot.teleoperators.utils import TeleopEvents
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, shutdown_rerun

logger = logging.getLogger(__name__)


@dataclass
class NeroPicoTeleoperateConfig:
    robot: RobotConfig
    teleop: TeleoperatorConfig
    fps: int = 30
    teleop_time_s: float | None = None
    display_data: bool = False
    display_cameras: bool = False
    use_orbbec_camera: bool = False
    orbbec_sn: str | None = None
    orbbec_width: int = 1280
    orbbec_height: int = 960
    orbbec_crop_width: int | None = None
    orbbec_crop_height: int | None = None
    orbbec_crop_x_offset: int | None = None
    orbbec_crop_y_offset: int | None = None
    use_realsense_camera: bool = False
    realsense_sn: str | None = None
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False


def _init_can():
    os.system("sudo ip link set can0 up type can bitrate 1000000 2>/dev/null")
    logger.info("CAN bus initialized (can0, 1Mbps)")


def _sync_ik_from_robot(teleop: NeroPicoVR, robot: Nero, retries: int = 10) -> None:
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


def nero_pico_teleop_loop(
    teleop: NeroPicoVR,
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

        robot.get_observation()
        raw_action = teleop.get_action()
        events = teleop.get_teleop_events()

        d = raw_action
        ev_s = bool(events.get(TeleopEvents.SUCCESS))
        ev_t = bool(events.get(TeleopEvents.TERMINATE_EPISODE))
        grip_d = d.get("delta_gripper", 0)
        try:
            raw_a = int(bool(xrt.get_A_button()))
            raw_b = int(bool(xrt.get_B_button()))
        except Exception:
            raw_a = raw_b = -1
        if grip_d != 0 or ev_s or ev_t or raw_a or raw_b or any(abs(d[k]) > 1e-4 for k in ("delta_x", "delta_y", "delta_z", "delta_wx", "delta_wy", "delta_wz")):
            print(
                f"  ACT: xy={d['delta_x']:+9.6f},{d['delta_y']:+9.6f},{d['delta_z']:+9.6f}  "
                f"wxyz={d['delta_wx']:+9.6f},{d['delta_wy']:+9.6f},{d['delta_wz']:+9.6f}  "
                f"grip_d={grip_d:+5.2f}  grip_s={teleop.gripper_state:5.1f}  "
                f"A={raw_a} B={raw_b} Y={int(ev_s)} back={int(ev_t)}"
            )

        if events.get(TeleopEvents.SUCCESS):
            logger.info("Y pressed — moving to Home position")
            going_home = True
            robot.move_to_home()
            _sync_ik_from_robot(teleop, robot)
            teleop.gripper_state = 100.0
            going_home = False
            continue

        if events.get(TeleopEvents.TERMINATE_EPISODE):
            logger.info("Left menu pressed — E-STOP, moving to Home then disconnecting")
            robot.move_to_home()
            return

        if teleop.is_home_requested():
            logger.info("X pressed — reconnecting robot")
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
def teleoperate(cfg: NeroPicoTeleoperateConfig):
    init_logging()

    if cfg.use_orbbec_camera:
        cfg.robot.cameras["top"] = OrbbecCameraConfig(
            fps=30, width=cfg.orbbec_width, height=cfg.orbbec_height,
            color_mode=ColorMode.RGB, warmup_s=2,
            auto_exposure=True, auto_white_balance=True,
            serial_number=cfg.orbbec_sn,
            crop_width=cfg.orbbec_crop_width, crop_height=cfg.orbbec_crop_height,
            crop_x_offset=cfg.orbbec_crop_x_offset, crop_y_offset=cfg.orbbec_crop_y_offset,
        )
        logger.info(f"Orbbec camera attached as 'top' (SN={cfg.orbbec_sn or 'auto'}, {cfg.orbbec_width}x{cfg.orbbec_height} -> {cfg.orbbec_crop_width or cfg.orbbec_width}x{cfg.orbbec_crop_height or cfg.orbbec_height})")

    if cfg.use_realsense_camera:
        sn = cfg.realsense_sn or ""
        cfg.robot.cameras["realsense"] = RealSenseCameraConfig(
            serial_number_or_name=sn,
            fps=30, width=640, height=480,
            color_mode=ColorMode.RGB,
            warmup_s=5,
        )
        logger.info(f"RealSense camera attached (SN={sn or 'auto'}, warmup=5s)")

    logging.info(pformat(asdict(cfg)))

    _init_can()

    if cfg.display_data:
        init_rerun(session_name="nero_pico_teleoperation", ip=cfg.display_ip, port=cfg.display_port)
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
        nero_pico_teleop_loop(
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
