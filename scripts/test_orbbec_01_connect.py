"""Orbbec 相机连接与断开基础测试.

验证 pyorbbecsdk Pipeline 的创建、启动、停止生命周期。
不需要 LeRobot，直接调用 SDK API。

Usage:
    python scripts/test_orbbec_01_connect.py
"""
import pyorbbecsdk


def test_connect_disconnect():
    print("[1] 创建 Pipeline...")
    pipeline = pyorbbecsdk.Pipeline()

    print("[2] 创建 Config，启用彩色流...")
    config = pyorbbecsdk.Config()

    color_profiles = pipeline.get_stream_profile_list(pyorbbecsdk.OBSensorType.COLOR_SENSOR)
    color_profile = color_profiles.get_default_video_stream_profile()
    config.enable_stream(color_profile)

    print("[3] 启动 Pipeline...")
    pipeline.start(config)
    print("[OK] Pipeline 启动成功！相机已连接。")

    print("[4] 停止 Pipeline...")
    pipeline.stop()
    print("[OK] Pipeline 已停止。相机已断开。")


if __name__ == "__main__":
    test_connect_disconnect()
