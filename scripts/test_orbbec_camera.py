"""Orbbec RGB + Depth 对齐流实时显示.

直接调用 pyorbbecsdk SDK（不经过 LeRobot），实时显示 RGB 和深度图伪彩色画面。
按 'q' 退出。用于验证 SDK 底层 RGB+Depth 对齐流是否正常。

Usage:
    python scripts/test_orbbec_camera.py [SN]
    # SN 省略时打开第一台相机; 双相机时建议显式指定:
    python scripts/test_orbbec_camera.py CL87B53009Z   # Femto Bolt
    python scripts/test_orbbec_camera.py CP0F4630000K  # Gemini 335
"""
import sys
import cv2
import numpy as np
from pyorbbecsdk import Context, Pipeline, Config, OBSensorType, OBFormat, OBAlignMode

def _pick_profile(profile_list, candidates, sensor_name):
    """按候选列表依次尝试, 找不到时回退默认 profile (SDK 找不到会抛 OBError 而非返回 None)."""
    for w, h, fmt, fps in candidates:
        try:
            p = profile_list.get_video_stream_profile(w, h, fmt, fps)
            if p is not None:
                print(f"[Info] {sensor_name}: 使用 {w}x{h}@{fps} {fmt}")
                return p
        except Exception:
            continue
    print(f"[Warning] {sensor_name}: 无匹配候选档位, 回退默认 profile")
    return profile_list.get_default_video_stream_profile()

def test_sdk_camera_robust():
    sn = sys.argv[1] if len(sys.argv) > 1 else None
    print(f"[Info] 正在初始化 Orbbec SDK 管道... (SN={sn or '第一台设备'})")
    if sn:
        ctx = Context()
        device_list = ctx.query_devices()
        device = None
        for i in range(device_list.get_count()):
            if device_list.get_device_by_index(i).get_device_info().get_serial_number() == sn:
                device = device_list.get_device_by_index(i)
                break
        if device is None:
            print(f"[Error] 未找到 SN={sn} 的设备")
            return
        pipeline = Pipeline(device)
    else:
        pipeline = Pipeline()
    config = Config()

    try:
        # 配置彩色流与深度流 (不同型号支持的档位不同, 逐档回退)
        profile_list_color = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = _pick_profile(profile_list_color, [
            (1280, 720, OBFormat.RGB, 30),
            (640, 480, OBFormat.RGB, 30),
        ], "彩色流")
        config.enable_stream(color_profile)

        profile_list_depth = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = _pick_profile(profile_list_depth, [
            (640, 576, OBFormat.Y16, 30),   # Femto Bolt
            (640, 400, OBFormat.Y16, 30),   # Gemini 335
            (640, 480, OBFormat.Y16, 30),
        ], "深度流")
        config.enable_stream(depth_profile)

        config.set_align_mode(OBAlignMode.SW_MODE)
        pipeline.start(config)
        print("[Success] SDK 管道启动成功。按 'q' 退出。")

    except Exception as e:
        print(f"[Error] 相机配置或启动失败: {e}")
        return

    frame_count = 0
    try:
        while True:
            # 增加等待超时时间至 200ms 以应对瞬时总线抖动
            frames = pipeline.wait_for_frames(200)
            
            if frames is None:
                print("[Warning] 帧读取超时 (Timeout)，底层缓冲池可能阻塞或总线断流...")
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame is None or depth_frame is None:
                continue

            # ================= 核心修复：执行内存深拷贝 =================
            # 获取底层指针后立即 copy()，解除对 C++ 底层 Frame 缓冲区的占用
            color_data = np.asanyarray(color_frame.get_data())
            color_image = np.copy(color_data.reshape((color_frame.get_height(), color_frame.get_width(), 3)))

            depth_data = np.asanyarray(depth_frame.get_data())
            depth_image = np.copy(np.frombuffer(depth_data, dtype=np.uint16).reshape(
                (depth_frame.get_height(), depth_frame.get_width())
            ))
            # ==========================================================

            # 颜色空间转换与深度图伪彩色处理
            color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 渲染显示
            win = sn or "Orbbec"
            cv2.imshow(f'{win} - RGB Stream (SDK)', color_image_bgr)
            cv2.imshow(f'{win} - Depth Stream (SDK)', depth_colormap)

            frame_count += 1
            if frame_count % 30 == 0:
                print(f"[Info] 持续接收视频流中，已成功渲染 {frame_count} 帧...")

            # 控制渲染刷新频率并监听退出指令
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[Info] 接收到退出指令...")
                break

    except KeyboardInterrupt:
        print("\n[Info] 程序被中断。")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[Info] SDK 硬件资源已安全释放。")

if __name__ == "__main__":
    test_sdk_camera_robust()