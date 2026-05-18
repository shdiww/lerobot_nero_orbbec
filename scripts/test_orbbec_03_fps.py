"""测试 3: Orbbec 相机连续读帧 + 帧率测量"""
import time
import numpy as np
import pyorbbecsdk
from pyorbbecsdk import OBPropertyID, OBPermissionType


def test_continuous_read():
    print("[1] 创建 Pipeline + 开启自动曝光/白平衡...")
    pipeline = pyorbbecsdk.Pipeline()
    device = pipeline.get_device()

    if device.is_property_supported(
        OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL,
        OBPermissionType.PERMISSION_READ_WRITE,
    ):
        device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, True)
    if device.is_property_supported(
        OBPropertyID.OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL,
        OBPermissionType.PERMISSION_READ_WRITE,
    ):
        device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, True)

    print("[2] 配置彩色流 1280x720 RGB@30fps...")
    config = pyorbbecsdk.Config()
    color_profiles = pipeline.get_stream_profile_list(pyorbbecsdk.OBSensorType.COLOR_SENSOR)
    color_profile = color_profiles.get_video_stream_profile(1280, 720, pyorbbecsdk.OBFormat.RGB, 30)
    if color_profile is None:
        color_profile = color_profiles.get_default_video_stream_profile()
    config.enable_stream(color_profile)

    print("[3] 启动 Pipeline...")
    pipeline.start(config)

    print("[4] 丢弃前 10 帧预热...")
    for _ in range(10):
        pipeline.wait_for_frames(1000)

    print("[5] 连续读取 100 帧并计时...")
    num_frames = 100
    timeout_count = 0
    t_start = time.perf_counter()

    for i in range(num_frames):
        frames = pipeline.wait_for_frames(1000)
        if frames is None:
            timeout_count += 1
            continue
        color_frame = frames.get_color_frame()
        if color_frame is None:
            timeout_count += 1
            continue
        _ = np.copy(np.asanyarray(color_frame.get_data()).reshape(
            (color_frame.get_height(), color_frame.get_width(), 3)
        ))

    t_elapsed = time.perf_counter() - t_start
    fps = num_frames / t_elapsed

    print(f"    总帧数: {num_frames}")
    print(f"    超时次数: {timeout_count}")
    print(f"    总耗时: {t_elapsed:.2f}s")
    print(f"    平均 FPS: {fps:.1f}")
    print(f"    目标 FPS: 30")

    if timeout_count > 0:
        print("[WARN] 有帧超时，可能是 USB 带宽不足或 SDK 缓冲问题")
    elif fps < 25:
        print("[WARN] FPS 偏低，可能需要调整分辨率或格式")
    else:
        print("[OK] 帧率正常！")

    print("[6] 停止 Pipeline...")
    pipeline.stop()
    print("[OK] 测试完成。")


if __name__ == "__main__":
    test_continuous_read()
