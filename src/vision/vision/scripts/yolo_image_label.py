import os
import cv2
import time
import numpy as np
import sys
# 将当前脚本所在目录加入到 sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from FootballDetection import FootballDetection 
import collections

# 定义类别颜色映射
# 定义类别颜色映射（BGR格式）
CATEGORY_COLORS = {
    "ball": {"box": (0, 0, 255), "font": (0, 0, 255)},         # 红色
    "goalpost": {"box": (230, 216, 173), "font": (230, 216, 173)}, # 浅蓝色（修正BGR顺序）
    "Person": {"box": (0, 255, 0), "font": (0, 255, 0)},       # 绿色
    "L": {"box": (0, 165, 255), "font": (0, 165, 255)},        # 橙色（BGR格式）
    "T": {"box": (0, 165, 255), "font": (0, 165, 255)},        # 橙色
    "X": {"box": (0, 165, 255), "font": (0, 165, 255)},        # 橙色
    "PenaltyPoint": {"box": (128, 0, 128), "font": (128, 0, 128)}, # 紫色
    "Opponent": {"box": (0, 255, 255), "font": (0, 255, 255)}, # 黄色
    "BRMarker": {"box": (255, 192, 203), "font": (255, 192, 203)}, # 粉色
    "Unknown": {"box": (128, 128, 128), "font": (128, 128, 128)} # 灰色
}

# 简化的颜色映射（可选）
BOX_COLORS = {
    k: v["box"] for k, v in CATEGORY_COLORS.items()
}

FONT_COLORS = {
    k: v["font"] for k, v in CATEGORY_COLORS.items()
}
# ================  1. 定义检测类（简化版本，去掉深度计算） ================ #
class Football_Detection:
    def __init__(self, model_path, conf_thres=0.5, iou_thres=0.5):
        # 不再需要相机内参
        self.detector = FootballDetection(model_path, conf_thres=conf_thres, iou_thres=iou_thres)

    def inference(self, image):
        """
        只返回 2D 检测结果，无 3D 相关计算
        返回格式：{class_id: [{"box": [x1,y1,x2,y2], "score": float}, ...], ...}
        """
        boxes, scores, class_ids = self.detector(image)  # 你的 FootballDetection 返回结果
        inference_message = {}
        for class_id, box, score in zip(class_ids, boxes, scores):
            if class_id not in inference_message:
                inference_message[class_id] = []
            inference_message[class_id].append({
                "box": box,
                "score": score
            })
        return inference_message

# ================  2. 画框显示的函数，去掉 3D 相关显示 ================ #
def display_inference_results(image, inference_dic, class_names):
    """
    在图像上画框并显示。
    """
    occupied_areas = []  # 记录已使用的标签区域

    for class_id, detections in inference_dic.items():
        for detection in detections:
            x1, y1, x2, y2 = detection["box"]
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

            # 获取类别名称和对应颜色
            class_name = class_names.get(class_id, "Unknown")
            box_color = BOX_COLORS.get(class_name, (128, 128, 128))  # 默认灰色
            font_color = FONT_COLORS.get(class_name, (255, 255, 255))  # 默认白色

            # 画框
            cv2.rectangle(image, (x1, y1), (x2, y2), box_color, 2)

            # 准备标签
            label = f"{class_name}: {detection['score']:.2f}"
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            text_x, text_y = x1, y1 - 10

            # 动态调整标签位置
            text_bg_x1, text_bg_y1 = text_x, text_y - text_size[1] - 5
            text_bg_x2, text_bg_y2 = text_x + text_size[0] + 5, text_y + 5

            # 如果标签位置超出图像顶部，放到框下方
            if text_bg_y1 < 0:
                text_y = y2 + 15
                text_bg_y1 = text_y - text_size[1] - 5
                text_bg_y2 = text_y + 5

            # # 确保标签不与其他标签或框重叠
            # while any(text_bg_x1 < area[2] and text_bg_x2 > area[0] and text_bg_y1 < area[3] and text_bg_y2 > area[1] for area in occupied_areas):
            #     text_y += 15  # 向下移动标签
            #     text_bg_y1 = text_y - text_size[1] - 5
            #     text_bg_y2 = text_y + 5

            # 保存占用区域
            occupied_areas.append((text_bg_x1, text_bg_y1, text_bg_x2, text_bg_y2))

            # 绘制灰色背景矩形
            cv2.rectangle(image, (text_bg_x1, text_bg_y1), (text_bg_x2, text_bg_y2), (50, 50, 50), -1)

            # 绘制文字
            cv2.putText(image, label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, font_color, 1)

    cv2.imshow("Detection Result", image)

def save_yolo_label(txt_path, inference_dic, image_width, image_height,classname_to_id,class_names):
    """
    将检测结果保存为 YOLO txt 格式。
    YOLO 格式：class_id x_center y_center width height （都需要归一化到 0~1）
    如果文件已存在，则追加写入
    """
    with open(txt_path, 'a') as f:
        for class_id, detections in inference_dic.items():
            for detection in detections:
                x1, y1, x2, y2 = detection["box"]
                # 转为 int 防止小数导致异常
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                bbox_width = x2 - x1
                bbox_height = y2 - y1
                center_x = x1 + bbox_width / 2
                center_y = y1 + bbox_height / 2

                # 归一化
                center_x /= image_width
                center_y /= image_height
                bbox_width /= image_width
                bbox_height /= image_height
                class_name =class_names[class_id]
                if class_name in classname_to_id:
                    
                    class_id_w = classname_to_id[class_name]
                    print(f"class_name: {class_name}, class_id: {class_id_w}")
                else:
                    continue
                # 写入：class_id cx cy w h
                f.write(f"{class_id_w} {center_x:.6f} {center_y:.6f} {bbox_width:.6f} {bbox_height:.6f}\n")


# ================  4. 主函数  ================ #
if __name__ == "__main__":
    # 模型路径
    model_path = "/home/unitree/Code/football_detect_cpp/weight/weight.onnx"
    classname_to_id = {
        "Ball": 0,
        "Goalpost": 1,
        "X": 2,
        "L": 3,
        "T": 4,
        "PenaltyPoint": 5,
        "person": 6,
        "Opponent": 7,
        "BRMarker": 8,
    }
    # 类别名称映射，可根据实际情况修改
    class_names = {
        0: "Ball",
        1: "Goalpost",
        2: "X",
        3: "L",
        4: "T"
    }
    footballdetection = Football_Detection(model_path)

    # 输入图像所在目录
    input_image_folder = "/home/unitree/newDisk/dataSet/RoboCupFootball_Dataset/trainset/images"         # 你要检测的原始图像存放的目录
    # 结果输出目录（用户确认后才存）
    output_image_folder = "/home/unitree/newDisk/dataSet/RoboCupFootball_Dataset/image_files2"   # 存放图像
    output_label_folder = "/home/unitree/newDisk/dataSet/RoboCupFootball_Dataset/label_files2"   # 存放txt标签

    # 创建输出目录（如果不存在）
    os.makedirs(output_image_folder, exist_ok=True)
    os.makedirs(output_label_folder, exist_ok=True)

    # 获取待检测图像列表
    image_files = sorted([f for f in os.listdir(input_image_folder) 
                          if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))])

    time_window = collections.deque(maxlen=30)  # 用于简单计算FPS

    for img_name in image_files:
        img_path = os.path.join(input_image_folder, img_name)

        # 读取图像
        color_image = cv2.imread(img_path)
        if color_image is None:
            print(f"Warning: could not read image {img_path}")
            continue

        start_time = time.time()

        # 推理，得到所有 bbox
        inference_dic = footballdetection.inference(color_image)

        # 在拷贝的图像上显示结果，避免覆盖原图
        display_image = color_image.copy()
        display_inference_results(display_image, inference_dic, class_names)
        key = cv2.waitKey(1)  # 等待用户按键

        # 简单计算 FPS
        elapsed = time.time() - start_time
        time_window.append(elapsed)
        if len(time_window) == time_window.maxlen:
            avg_elapsed = sum(time_window) / len(time_window)
            fps = 1.0 / avg_elapsed if avg_elapsed > 0 else 0.0
        else:
            fps = 0.0
        print(f"Current FPS (approx): {fps:.2f}")

        if key == ord('q'):
            # 如果按下 'q'，退出程序
            print("Quit without saving.")
            break
        elif key == ord('s'):
            # 如果按下 's'，保存图像和标签
            base_name, ext = os.path.splitext(img_name)
            save_img_path = os.path.join(output_image_folder, img_name)
            save_txt_path = os.path.join(output_label_folder, base_name + ".txt")

            # 保存检测结果到txt（YOLO格式）
            h, w = color_image.shape[:2]
            save_yolo_label(save_txt_path, inference_dic, w, h, classname_to_id,class_names)
            # 保存图像
            cv2.imwrite(save_img_path, color_image)
            print(f"Saved: {save_img_path} and {save_txt_path}")
        else:
            # 其他按键则跳过保存，继续下一个
            print("Skipped saving.")

    cv2.destroyAllWindows()
