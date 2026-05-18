import cv2
import numpy as np
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat, OBAlignMode

def test_sdk_camera_robust():
    print("[Info] 正在初始化 Orbbec SDK 管道...")
    pipeline = Pipeline()
    config = Config()

    try:
        # 配置彩色流与深度流
        profile_list_color = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = profile_list_color.get_video_stream_profile(1280, 720, OBFormat.RGB, 30)
        config.enable_stream(color_profile if color_profile else profile_list_color.get_default_video_stream_profile())

        profile_list_depth = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = profile_list_depth.get_video_stream_profile(640, 576, OBFormat.Y16, 30)
        config.enable_stream(depth_profile if depth_profile else profile_list_depth.get_default_video_stream_profile())

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
            cv2.imshow('Femto Bolt - RGB Stream (SDK)', color_image_bgr)
            cv2.imshow('Femto Bolt - Depth Stream (SDK)', depth_colormap)

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