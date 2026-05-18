"""测试 6: OrbbecCamera 在 Nero 机器人框架下工作"""
import time
import cv2
import numpy as np
from lerobot.cameras.orbbec import OrbbecCamera, OrbbecCameraConfig
from lerobot.cameras.configs import ColorMode
from lerobot.cameras.utils import make_cameras_from_configs


def test_nero_camera_framework():
    print("=" * 60)
    print("测试 OrbbecCamera 在 Nero 机器人框架下工作")
    print("=" * 60)

    print("\n[1] 模拟 Nero 机器人的 cameras 配置...")
    camera_configs = {
        "top": OrbbecCameraConfig(
            fps=30,
            width=1280,
            height=720,
            color_mode=ColorMode.RGB,
            warmup_s=2,
            auto_exposure=True,
            auto_white_balance=True,
        ),
    }
    print(f"    camera_configs = {camera_configs}")

    print("\n[2] 通过 make_cameras_from_configs 工厂创建相机实例...")
    cameras = make_cameras_from_configs(camera_configs)
    print(f"    创建了 {len(cameras)} 个相机实例: {list(cameras.keys())}")
    assert "top" in cameras
    assert isinstance(cameras["top"], OrbbecCamera)
    print("    [OK] 工厂函数正确创建 OrbbecCamera")

    print("\n[3] 模拟 robot.connect() — 连接所有相机...")
    for name, cam in cameras.items():
        cam.connect(warmup=True)
        print(f"    {name}: is_connected={cam.is_connected}")

    print("\n[4] 模拟 robot.get_observation() — 读取相机帧...")
    obs = {}
    for name, cam in cameras.items():
        obs[name] = cam.read_latest(max_age_ms=2000)
        print(f"    {name}: shape={obs[name].shape}, dtype={obs[name].dtype}")

    assert obs["top"].shape == (720, 1280, 3)

    print("\n[5] 保存帧到文件...")
    frame_bgr = cv2.cvtColor(obs["top"], cv2.COLOR_RGB2BGR)
    cv2.imwrite("/tmp/orbbec_nero_framework_test.png", frame_bgr)
    print("    保存到 /tmp/orbbec_nero_framework_test.png")

    print("\n[6] 模拟遥操作循环 — 30 次循环读取 + 计时...")
    t0 = time.perf_counter()
    for i in range(30):
        for name, cam in cameras.items():
            frame = cam.read_latest(max_age_ms=2000)
    elapsed = time.perf_counter() - t0
    hz = 30 / elapsed
    print(f"    30 次循环耗时: {elapsed:.2f}s, 频率: {hz:.1f} Hz")
    if hz >= 20:
        print("    [OK] 频率满足遥操作需求 (>= 20 Hz)")
    else:
        print("    [WARN] 频率偏低，可能影响遥操作")

    print("\n[7] 模拟 robot.disconnect() — 断开所有相机...")
    for name, cam in cameras.items():
        cam.disconnect()
        print(f"    {name}: is_connected={cam.is_connected}")

    print("\n" + "=" * 60)
    print("全部通过！OrbbecCamera 可在 Nero 机器人框架下正常工作。")
    print("=" * 60)


if __name__ == "__main__":
    test_nero_camera_framework()
