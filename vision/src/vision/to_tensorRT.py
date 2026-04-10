from ultralytics import YOLO

MODEL_NAME = "yolo11n.pt"

model = YOLO(MODEL_NAME)  # or yolo11s, yolo26n, etc.
model.export(format="engine", device=0, half=True)  # FP16 is great on Orin Nano Super
