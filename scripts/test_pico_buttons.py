#!/usr/bin/env python
"""Pico VR 原始按键/摇杆/位姿数据读取测试.

直接调用 xrobotoolkit_sdk API（不经过 LeRobot），
读取并打印所有按键、摇杆、握柄、扳机、控制器位姿的原始值。
用于验证 SDK 底层数据通道是否正常。

Usage:
    python scripts/test_pico_buttons.py
"""

import time

import xrobotoolkit_sdk as xrt

xrt.init()
print("SDK OK. 按 A/B/X/Y/握柄/扳机/摇杆看值变化...\n")

try:
    while True:
        rp = xrt.get_right_controller_pose()
        lp = xrt.get_left_controller_pose()
        rg = xrt.get_right_grip()
        lg = xrt.get_left_grip()
        rt = xrt.get_right_trigger()
        lt = xrt.get_left_trigger()
        a = xrt.get_A_button()
        b = xrt.get_B_button()
        x = xrt.get_X_button()
        y = xrt.get_Y_button()
        rm = xrt.get_right_menu_button()
        lm = xrt.get_left_menu_button()
        rax = xrt.get_right_axis()
        lax = xrt.get_left_axis()
        rac = xrt.get_right_axis_click()
        lac = xrt.get_left_axis_click()

        def _pos(p):
            if isinstance(p, (list, tuple)) and len(p) >= 3:
                return f"{p[0]:.2f} {p[1]:.2f} {p[2]:.2f}"
            return "?"

        def _ax(a):
            if isinstance(a, (list, tuple)) and len(a) >= 2:
                return f"({a[0]:.2f},{a[1]:.2f})"
            return str(a)

        print(
            f"A={a} B={b} X={x} Y={y} "
            f"RM={rm} LM={lm} "
            f"grip={rg:.2f}/{lg:.2f} "
            f"trig={rt:.2f}/{lt:.2f} "
            f"joy={_ax(rax)}/{_ax(lax)} "
            f"click={rac}/{lac} "
            f"pos=({_pos(rp)})/({_pos(lp)})"
        )

        time.sleep(0.033)

except KeyboardInterrupt:
    print("\nDone.")
    xrt.close()
