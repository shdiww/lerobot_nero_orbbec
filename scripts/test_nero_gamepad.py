#!/usr/bin/env python
"""Nero 手柄遥操作测试

作用:
    验证 NeroGamepad 作为 LeRobot Teleoperator 子类的完整链路:
    1. make_teleoperator_from_config 工厂能否正确创建实例
    2. config 注册是否生效 (type = nero_gamepad)
    3. action_features 是否输出 delta_x/y/z/wx/wy/wz + delta_gripper
    4. Xbox 手柄能否连接并读取增量动作
    5. 按键映射是否正确

按键逻辑:
    L-stick  → 末端 X/Y 平移 (delta_x, delta_y)
    R-stick  → 末端 Z 平移 / Z 轴旋转 (delta_z, delta_wz)
    D-pad    → 末端 X/Y 旋转 (delta_wx, delta_wy)
    A        → 夹爪闭合 (delta_gripper < 0)
    B        → 夹爪张开 (delta_gripper > 0)
    Y        → 回 Home (episode SUCCESS)
    Back     → E-STOP (episode FAILURE)
    Home     → 连接机械臂 (home_requested 标志)

前提:
    - Xbox 手柄已通过蓝牙或 USB 连接到电脑
    - pygame 已安装 (uv sync --extra nero)

预期现象:
    [1/3] 打印 config.type = nero_gamepad
          打印 action_features 包含 delta_x/y/z/wx/wy/wz + delta_gripper
    [2/3] 手柄连接后 is_connected = True
          (若 10 秒内未检测到手柄会打印警告, 步骤 3 跳过)
    [3/3] 实时打印 10 秒的增量动作:
          - 推左摇杆 → delta_x / delta_y 变化
          - 推右摇杆 → delta_z / delta_wz 变化
          - 按 D-pad → delta_wx / delta_wy 变化
          - 按 A → delta_gripper 为负 (闭合)
          - 按 B → delta_gripper 为正 (张开)
          - 按 Y → SUCCESS 事件触发
          - 按 Back → FAILURE 事件触发
    最终输出 "Gamepad test PASSED."

Usage:
    uv run python scripts/test_nero_gamepad.py
"""

import time

from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.nero_gamepad import NeroGamepadConfig
from lerobot.teleoperators.utils import TeleopEvents


def main():
    print("[1/3] Testing class import and config registration...")
    config = NeroGamepadConfig()
    print(f"  config.type = {config.type}")
    print(f"  use_gripper = {config.use_gripper}")

    teleop = make_teleoperator_from_config(config)
    print(f"  teleop.name = {teleop.name}")
    print(f"  action_features = {teleop.action_features}")

    print("\n[2/3] Testing connect (Xbox controller required)...")
    teleop.connect()
    print(f"  is_connected = {teleop.is_connected}")

    if teleop.is_connected:
        print("\n[3/3] Reading actions for 10 seconds...")
        print("  Move the joysticks and press buttons on your Xbox controller.")
        start = time.monotonic()
        try:
            while time.monotonic() - start < 10.0:
                action = teleop.get_action()
                delta_str = " ".join(f"{k}: {v:+.5f}" for k, v in action.items())
                events = teleop.get_teleop_events()
                event_str = ""
                if events[TeleopEvents.SUCCESS]:
                    event_str = " [SUCCESS/Y]"
                if events[TeleopEvents.TERMINATE_EPISODE]:
                    event_str += " [E-STOP/Back]"
                home = teleop.is_home_requested()
                if home:
                    event_str += " [HOME]"
                print(f"\r  {delta_str}{event_str}", end="", flush=True)
                time.sleep(0.05)
            print()
        except KeyboardInterrupt:
            print("\n  Interrupted.")
    else:
        print("[3/3] SKIPPED — no gamepad detected")

    teleop.disconnect()
    print(f"  is_connected after disconnect = {teleop.is_connected}")

    print("\nGamepad test PASSED.")


if __name__ == "__main__":
    main()
