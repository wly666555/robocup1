from ultralytics import YOLO

# Load the YOLO model
model = YOLO("/home/unitree/Code/football_detect_cpp/weight/yolo11x.pt")

# Export the model to ONNX format
# export_path = model.export(format="onnx")
# 导出 ONNX，避免 INT64
export_path = model.export(
    format="onnx",
    dynamic=False,  # 禁用动态轴（减少 INT64 风险）
    half=False,     # 禁用 FP16，避免 subnormal values
)

print(f"Model exported to {export_path}")
