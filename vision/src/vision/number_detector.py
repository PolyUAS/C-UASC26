"""
Number identification for target crops.

Use mode='manual' for a human-entered number during testing.
Use mode='auto' with a trained YOLOv8 classifier/detector model for autonomous runs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class NumberResult:
    number: str | None
    confidence: float
    source: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "number": self.number,
            "confidence": self.confidence,
            "source": self.source,
        }


class NumberDetector:
    def __init__(self, mode: str = "manual", model_path: str | None = None, confidence_threshold: float = 0.50, device: str | None = None):
        self.mode = mode
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.device = device
        self.model = None

        if mode not in {"manual", "auto"}:
            raise ValueError("mode must be 'manual' or 'auto'")

        if mode == "auto":
            if model_path is None:
                raise ValueError("auto mode requires model_path for the number model")
            self._load_auto_model(model_path)

    def _load_auto_model(self, model_path: str) -> None:
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError("Auto number detection requires torch and ultralytics.") from exc

        self.device = self.device or ("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model = YOLO(model_path)
        print(f"NumberDetector device: {self.device}")

    def identify(self, crop, manual_number: str | None = None) -> dict[str, Any]:
        if self.mode == "manual":
            return NumberResult(number=manual_number, confidence=1.0 if manual_number else 0.0, source="manual").to_dict()
        return self._identify_auto(crop).to_dict()

    def _identify_auto(self, crop) -> NumberResult:
        if self.model is None:
            return NumberResult(number=None, confidence=0.0, source="auto")

        results = self.model.predict(source=crop, device=self.device, conf=self.confidence_threshold, verbose=False)
        if not results:
            return NumberResult(number=None, confidence=0.0, source="auto")

        result = results[0]
        names = result.names or self.model.names

        # Classification model path: result.probs exists.
        if getattr(result, "probs", None) is not None and result.probs is not None:
            top_index = int(result.probs.top1)
            confidence = float(result.probs.top1conf)
            number = str(names[top_index]) if top_index in names else str(top_index)
            return NumberResult(number=number, confidence=confidence, source="auto")

        # Detection model path: choose highest-confidence detected digit/number.
        if result.boxes is not None and len(result.boxes) > 0:
            best_box = max(result.boxes, key=lambda b: float(b.conf[0]))
            confidence = float(best_box.conf[0])
            class_index = int(best_box.cls[0])
            number = str(names[class_index]) if class_index in names else str(class_index)
            return NumberResult(number=number, confidence=confidence, source="auto")

        return NumberResult(number=None, confidence=0.0, source="auto")
