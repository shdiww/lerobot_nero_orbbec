#!/usr/bin/env python
"""Nero 机器人连接测试

作用:
    验证 Nero 作为 LeRobot Robot 子类的生命周期:
    1. connect() 后 is_connected 应为 True
    2. get_observation() 能否读到 7 个关节角 + gripper (如有)
    3. move_to_home() 后关节角度接近 home_joint_angles
    4. send_action 测试夹爪开合 (关节保持 home, 夹爪开→关)
    5. disconnect() 后 is_connected 应为 False
    6. 断开后能否重新 connect

前提:
    - CAN 接口已启用 (sudo ip link set can0 up type can bitrate 1000000)
    - Nero 机械臂已上电并连接 CAN 总线
    - pyAgxArm 已安装 (cd pyAgxArm && uv pip install -e .)

预期现象:
    [1/6] is_connected = True
    [2/6] 打印 observation keys (joint1.pos ~ joint7.pos, gripper.pos)
          以及各关节当前弧度值
    [3/6] move_to_home() 后各关节角度接近 home_joint_angles
    [4/6] send_action 发送 home + gripper open, 等待夹爪打开;
          再发送 home + gripper close, 等待夹爪闭合
    [5/6] 打印 is_connected = False
    [6/6] 重新连接后 is_connected = True, 再次断开后 = False
    最终输出 "All connect/disconnect tests PASSED."

Usage:
    uv run python scripts/test_nero_connect.py
"""
import time
from lerobot.robots import make_robot_from_config
from lerobot.robots.nero import NeroRobotConfig
from lerobot.robots.nero.config_nero import NERO_GRIPPER_MAX_WIDTH_M, NERO_JOINT_NAMES


def _wait_joints_at_home(robot, home_angles, timeout=10.0):
    t0 = time.monotonic()
    while True:
        obs = robot.get_observation()
        if all(abs(obs[f"{name}.pos"] - home_angles[i]) < 0.1 for i, name in enumerate(NERO_JOINT_NAMES)):
            break
        if time.monotonic() - t0 > timeout:
            break
        time.sleep(0.1)
    return obs


def _wait_gripper(robot, target, tol=0.005, timeout=5.0):
    t0 = time.monotonic()
    while True:
        obs = robot.get_observation()
        val = obs.get("gripper.pos", 0.0)
        if abs(val - target) < tol:
            break
        if time.monotonic() - t0 > timeout:
            break
        time.sleep(0.1)
    return obs


def main():
    robot = make_robot_from_config(NeroRobotConfig())
    timeout = 10.0

    print("[1/6] Testing connect...")
    robot.connect(calibrate=False)
    print(f"  is_connected = {robot.is_connected}")
    assert robot.is_connected, "Robot should be connected after connect()"

    print("[2/6] Testing get_observation...")
    obs = robot.get_observation()
    print(f"  observation keys: {list(obs.keys())}")
    for k, v in obs.items():
        if isinstance(v, float):
            print(f"    {k}: {v:.4f}")

    print("[3/6] Testing go home...")
    robot.move_to_home()
    home_angles = robot.config.home_joint_angles
    obs = _wait_joints_at_home(robot, home_angles, timeout)
    for i, name in enumerate(NERO_JOINT_NAMES):
        actual = obs[f"{name}.pos"]
        assert abs(actual - home_angles[i]) < 0.1, f"{name}: expected {home_angles[i]:.4f}, got {actual:.4f}"
    print("  Joints at home position.")

    if robot.config.use_gripper:
        print("[4/6] Testing gripper via send_action...")
        gripper_action = {"gripper.pos": NERO_GRIPPER_MAX_WIDTH_M}
        robot.send_action(gripper_action)
        obs = _wait_gripper(robot, NERO_GRIPPER_MAX_WIDTH_M, tol=0.01, timeout=5.0)
        gripper_val = obs["gripper.pos"]
        assert gripper_val > 0.01, f"gripper: expected >0.01 after open, got {gripper_val:.6f}"
        print(f"  gripper.pos = {gripper_val:.6f} (open)")

        time.sleep(0.5)
        gripper_action = {"gripper.pos": 0.0}
        robot.send_action(gripper_action)
        obs = _wait_gripper(robot, 0.0, tol=0.005, timeout=5.0)
        gripper_val = obs["gripper.pos"]
        assert abs(gripper_val) < 0.005, f"gripper: expected ≈0.0 after close, got {gripper_val:.6f}"
        print(f"  gripper.pos = {gripper_val:.6f} (closed)")
    else:
        print("[4/6] Skipping gripper test")

    print("[5/6] Testing disconnect...")
    robot.disconnect()
    print(f"  is_connected = {robot.is_connected}")
    assert not robot.is_connected, "Robot should not be connected after disconnect()"

    print("[6/6] Testing reconnect...")
    robot.connect(calibrate=False)
    assert robot.is_connected, "Robot should be re-connectable"
    robot.disconnect()
    assert not robot.is_connected

    print("\nAll connect/disconnect tests PASSED.")


if __name__ == "__main__":
    main()
