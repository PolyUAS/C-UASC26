"""
YOLOv8 target detector wrapper.

Input:  OpenCV frame from the RTSP stream.
Output: list of detection dictionaries compatible with the generated Detection.py
        FlatBuffer fields: ctrX, ctrY, width, height, classId, metadata, confidence.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class TargetDetection:
    ctrX: int
    ctrY: int
    width: int
    height: int
    classId: str
    metadata: str
    confidence: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "ctrX": self.ctrX,
            "ctrY": self.ctrY,
            "width": self.width,
            "height": self.height,
            "classId": self.classId,
            "metadata": self.metadata,
            "confidence": self.confidence,
        }


class TargetDetector:
    """Loads one YOLOv8 model and runs target detection on OpenCV frames."""

    def __init__(self, model_path: str, confidence_threshold: float = 0.25, device: str | None = None):
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError(
                "TargetDetector requires torch and ultralytics. "
                "Install them in your uv environment before running detection."
            ) from exc

        self.torch = torch
        self.device = device or ("cuda:0" if torch.cuda.is_available() else "cpu")
        self.confidence_threshold = confidence_threshold
        self.model = YOLO(model_path)

        print(f"TargetDetector device: {self.device}")
        if torch.cuda.is_available():
            print(f"CUDA device: {torch.cuda.get_device_name(0)}")
        else:
            print("CUDA is not available; YOLO will run on CPU.")

    def detect(self, frame) -> list[dict[str, Any]]:
        """Run YOLOv8 detection on a single OpenCV BGR frame."""
        results = self.model.predict(
            source=frame,
            device=self.device,
            conf=self.confidence_threshold,
            verbose=False,
        )

        detections: list[dict[str, Any]] = []

        for result in results:
            if result.boxes is None:
                continue

            names = result.names or self.model.names

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = float(box.conf[0])
                class_index = int(box.cls[0])
                class_name = str(names[class_index]) if class_index in names else str(class_index)

                width = max(0, x2 - x1)
                height = max(0, y2 - y1)
                ctr_x = x1 + width / 2
                ctr_y = y1 + height / 2

                detections.append(
                    TargetDetection(
                        ctrX=int(round(ctr_x)),
                        ctrY=int(round(ctr_y)),
                        width=int(round(width)),
                        height=int(round(height)),
                        classId=class_name,
                        metadata="",
                        confidence=confidence,
                    ).to_dict()
                )

        return detections


def print_cuda_status() -> bool:
    """Small CUDA test helper for Jetson or desktop development."""
    try:
        import torch
    except ImportError:
        print("torch is not installed in this environment.")
        return False

    print(f"Torch version: {torch.__version__}")
    available = torch.cuda.is_available()
    print(f"CUDA available: {available}")
    if available:
        print(f"CUDA device count: {torch.cuda.device_count()}")
        print(f"CUDA device 0: {torch.cuda.get_device_name(0)}")
    return available


if __name__ == "__main__":
    print_cuda_status()
