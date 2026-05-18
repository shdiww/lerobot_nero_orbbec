"""测试 4: Orbbec 相机后台线程 + async_read 模式"""
import time
import threading
import numpy as np
import pyorbbecsdk
from pyorbbecsdk import OBPropertyID, OBPermissionType


def test_async_read():
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

    # === 后台线程相关变量 ===
    latest_frame = None
    latest_timestamp = 0.0
    new_frame_event = threading.Event()
    frame_lock = threading.Lock()
    stop_event = threading.Event()

    def _read_loop():
        nonlocal latest_frame, latest_timestamp
        while not stop_event.is_set():
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            img = np.copy(np.asanyarray(color_frame.get_data()).reshape(
                (color_frame.get_height(), color_frame.get_width(), 3)
            ))
            with frame_lock:
                latest_frame = img
                latest_timestamp = time.perf_counter()
            new_frame_event.set()

    print("[4] 启动后台读帧线程...")
    read_thread = threading.Thread(target=_read_loop, daemon=True)
    read_thread.start()

    # === 等第一帧到达 ===
    print("[5] 等待第一帧...")
    if not new_frame_event.wait(timeout=5.0):
        print("[FAIL] 5秒内没收到帧！")
        stop_event.set()
        read_thread.join(timeout=2.0)
        pipeline.stop()
        return

    with frame_lock:
        first_frame = latest_frame
        first_ts = latest_timestamp
    print(f"    第一帧到达, shape={first_frame.shape}, ts={first_ts:.3f}")

    # === async_read 测试：主线程等新帧 ===
    print("[6] 测试 async_read 模式（主线程等新帧通知）...")
    num_reads = 50
    latencies = []
    for i in range(num_reads):
        new_frame_event.clear()
        if not new_frame_event.wait(timeout=1.0):
            print(f"    [WARN] 第 {i+1} 次 async_read 超时")
            continue
        with frame_lock:
            frame_ts = latest_timestamp
        latency = (time.perf_counter() - frame_ts) * 1000
        latencies.append(latency)

    if latencies:
        print(f"    读取次数: {len(latencies)}")
        print(f"    平均延迟: {sum(latencies)/len(latencies):.1f} ms")
        print(f"    最大延迟: {max(latencies):.1f} ms")
        print(f"    最小延迟: {min(latencies):.1f} ms")
    else:
        print("[FAIL] 没有成功读取任何帧")

    # === 停止 ===
    print("[7] 停止后台线程和 Pipeline...")
    stop_event.set()
    read_thread.join(timeout=2.0)
    pipeline.stop()
    print("[OK] 测试完成。")


if __name__ == "__main__":
    test_async_read()
