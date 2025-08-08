import pyrealsense2 as rs
import os
import time
import cv2
import collections
import numpy as np

if __name__ == '__main__':
    output_dir = "./saved_frames"
    os.makedirs(output_dir, exist_ok=True)  # 创建保存图像的目录

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)

    # Get camera intrinsics
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align = rs.align(rs.stream.color)
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()


    last_time = time.time()
    time_window = collections.deque(maxlen=30)  # 用于 FPS 计算

    frame_counter = 0  # 帧计数器

    while True:
        start_time = time.time()

        frameset = pipeline.wait_for_frames()
        aligned_frameset = align.process(frameset)
        color_frame = aligned_frameset.get_color_frame()
        depth_frame = aligned_frameset.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # 每 5 帧保存一次图像
        frame_counter += 1
        if frame_counter % 5 == 0:
            timestamp = int(time.time() * 1000)
            save_path = os.path.join(output_dir, f"frame_{timestamp}.png")
            cv2.imwrite(save_path, color_image)
            print(f"Saved frame to {save_path}")

        # 计算 FPS
        end_time = time.time()
        elapsed = end_time - start_time
        time_window.append(elapsed)

        if len(time_window) == time_window.maxlen:
            avg_elapsed = sum(time_window) / len(time_window)
            fps = 1.0 / avg_elapsed if avg_elapsed > 0 else 0.0
        else:
            fps = 0.0

        print(f"FPS: {fps:.2f}")

        # 显示图像
        cv2.imshow("Color Frame", color_image)
        key = cv2.waitKey(1)

        if key == ord('q'):  # 按下 'q' 键退出
            break

    pipeline.stop()
