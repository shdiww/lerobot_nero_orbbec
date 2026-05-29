#!/usr/bin/env python
"""用 PicoVRController 类测试按键 + 6D delta (与遥操作完全相同的代码路径)."""

import sys
import time

sys.path.insert(0, "lerobot/src")

import xrobotoolkit_sdk as xrt

from lerobot.teleoperators.pico_vr.configuration_pico_vr import NeroPicoVRConfig
from lerobot.teleoperators.pico_vr.pico_vr_utils import PicoVRController

cfg = NeroPicoVRConfig()
ctrl = PicoVRController(cfg)

print("Connecting PicoVRController...")
ctrl.start()
print("Connected.\n")
print("Grip (=握柄) 激活 6D 追踪, 松开则 delta 归零.")
print("按 A/B/X/Y/Menu 看事件, Ctrl+C 退出.")
print("=" * 120)

last_update = 0

try:
    while True:
        now = time.time()

        # 完全相同的调用链: get_action() -> update() -> _update_buttons() + _update_tracking()
        ctrl.update()

        dx, dy, dz, dwx, dwy, dwz = ctrl.get_deltas()
        grip_delta = ctrl.gripper_command()

        raw_a = bool(xrt.get_A_button())
        raw_b = bool(xrt.get_B_button())
        raw_x = bool(xrt.get_X_button())
        raw_y = bool(xrt.get_Y_button())
        raw_rm = bool(xrt.get_right_menu_button())
        raw_lm = bool(xrt.get_left_menu_button())
        raw_rg = xrt.get_right_grip()

        home_req = ctrl.is_home_requested()
        ep_status = ctrl.get_episode_end_status()
        start_p = ctrl.is_start_pressed()
        back_p = ctrl.is_back_pressed()

        has_delta = any(abs(v) > 1e-6 for v in (dx, dy, dz, dwx, dwy, dwz))
        has_event = grip_delta != 0.0 or home_req or ep_status is not None or start_p or back_p

        if has_delta or has_event or now - last_update > 1.0:
            print(
                f"grip={raw_rg:.2f}  "
                f"delta: x={dx:+7.4f} y={dy:+7.4f} z={dz:+7.4f}  "
                f"wx={dwx:+7.4f} wy={dwy:+7.4f} wz={dwz:+7.4f}  |  "
                f"A={int(raw_a)} B={int(raw_b)} X={int(raw_x)} Y={int(raw_y)} "
                f"RM={int(raw_rm)} LM={int(raw_lm)}  |  "
                f"grip_delta={grip_delta:+.1f} home={int(home_req)} "
                f"ep={ep_status or '-'} start={int(start_p)} back={int(back_p)}"
            )
            last_update = now

        time.sleep(0.033)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    ctrl.stop()
    print("Done.")
