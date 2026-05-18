"""测试 5: OrbbecCamera LeRobot 集成验证"""
import cv2
import numpy as np
from lerobot.cameras.orbbec import OrbbecCamera, OrbbecCameraConfig
from lerobot.cameras.configs import ColorMode


def test_lerobot_integration():
    print("=" * 60)
    print("测试 OrbbecCamera LeRobot 集成")
    print("=" * 60)

    print("\n[1] 测试 find_cameras()...")
    cameras = OrbbecCamera.find_cameras()
    print(f"    找到 {len(cameras)} 个 Orbbec 相机:")
    for c in cameras:
        print(f"      {c}")

    print("\n[2] 创建 OrbbecCamera 实例...")
    config = OrbbecCameraConfig(
        fps=30,
        width=1280,
        height=720,
        color_mode=ColorMode.RGB,
        warmup_s=2,
        auto_exposure=True,
        auto_white_balance=True,
    )
    cam = OrbbecCamera(config)
    print(f"    {cam}")

    print("\n[3] 测试 connect()...")
    cam.connect(warmup=True)
    print(f"    is_connected: {cam.is_connected}")

    print("\n[4] 测试 read() — 同步读取一帧...")
    frame = cam.read()
    print(f"    shape: {frame.shape}, dtype: {frame.dtype}")
    assert frame.shape == (720, 1280, 3), f"Unexpected shape: {frame.shape}"
    print("    [OK]")

    print("\n[5] 测试 async_read() — 异步读取一帧...")
    frame2 = cam.async_read(timeout_ms=1000)
    print(f"    shape: {frame2.shape}")
    assert frame2.shape == (720, 1280, 3)
    print("    [OK]")

    print("\n[6] 测试 read_latest() — 读取最新帧...")
    frame3 = cam.read_latest(max_age_ms=2000)
    print(f"    shape: {frame3.shape}")
    assert frame3.shape == (720, 1280, 3)
    print("    [OK]")

    print("\n[7] 保存一帧到文件验证颜色...")
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite("/tmp/orbbec_lerobot_test.png", frame_bgr)
    print("    保存到 /tmp/orbbec_lerobot_test.png")

    print("\n[8] 测试连续 read_latest 30 次测帧率...")
    import time
    t0 = time.perf_counter()
    for _ in range(30):
        cam.read_latest(max_age_ms=2000)
    elapsed = time.perf_counter() - t0
    print(f"    30 次 read_latest 耗时: {elapsed:.2f}s, 等效频率: {30/elapsed:.1f} Hz")

    print("\n[9] 测试 disconnect()...")
    cam.disconnect()
    print(f"    is_connected: {cam.is_connected}")
    print("    [OK]")

    print("\n" + "=" * 60)
    print("全部测试通过！OrbbecCamera 已成功集成到 LeRobot。")
    print("=" * 60)


if __name__ == "__main__":
    test_lerobot_integration()
