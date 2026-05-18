"""测试 2: Orbbec 相机读单帧 + 自动曝光/白平衡"""
import cv2
import numpy as np
import pyorbbecsdk
from pyorbbecsdk import OBPropertyID, OBPermissionType


def test_read_single_frame():
    print("[1] 创建 Pipeline...")
    pipeline = pyorbbecsdk.Pipeline()
    device = pipeline.get_device()

    print("[2] 开启彩色自动曝光...")
    if device.is_property_supported(
        OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL,
        OBPermissionType.PERMISSION_READ_WRITE,
    ):
        device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, True)
        print("    自动曝光: ON")
    else:
        print("    [WARN] 设备不支持自动曝光")

    print("[3] 开启彩色自动白平衡...")
    if device.is_property_supported(
        OBPropertyID.OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL,
        OBPermissionType.PERMISSION_READ_WRITE,
    ):
        device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, True)
        print("    自动白平衡: ON")
    else:
        print("    [WARN] 设备不支持自动白平衡")

    print("[4] 配置彩色流 1280x720 RGB@30fps...")
    config = pyorbbecsdk.Config()
    color_profiles = pipeline.get_stream_profile_list(pyorbbecsdk.OBSensorType.COLOR_SENSOR)
    color_profile = color_profiles.get_video_stream_profile(1280, 720, pyorbbecsdk.OBFormat.RGB, 30)
    if color_profile is None:
        color_profile = color_profiles.get_default_video_stream_profile()
    config.enable_stream(color_profile)

    print("[5] 启动 Pipeline...")
    pipeline.start(config)

    print("[6] 丢弃前 30 帧，等自动曝光/白平衡稳定...")
    for i in range(30):
        frames = pipeline.wait_for_frames(1000)
        if frames is None:
            print(f"    [WARN] 第 {i+1} 帧超时")

    print("[7] 读取一帧...")
    frames = pipeline.wait_for_frames(1000)
    if frames is None:
        print("[FAIL] 等待帧超时！")
        pipeline.stop()
        return

    color_frame = frames.get_color_frame()
    if color_frame is None:
        print("[FAIL] 没有彩色帧！")
        pipeline.stop()
        return

    color_data = np.asanyarray(color_frame.get_data())
    color_image = np.copy(color_data.reshape((color_frame.get_height(), color_frame.get_width(), 3)))

    print(f"    图像尺寸: {color_image.shape}, dtype: {color_image.dtype}")

    print("[8] RGB -> BGR 转换并保存...")
    color_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    save_path = "/tmp/orbbec_test_frame.png"
    cv2.imwrite(save_path, color_bgr)
    print(f"[OK] 图片已保存到: {save_path}")
    print("[提示] 请检查图片颜色是否正常（不偏蓝/不过曝/不过暗）")

    print("[9] 停止 Pipeline...")
    pipeline.stop()
    print("[OK] 测试完成。")


if __name__ == "__main__":
    test_read_single_frame()
