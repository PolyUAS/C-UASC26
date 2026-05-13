"""Store the best known target observation keyed by the target number."""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any


@dataclass
class TargetObservation:
    number: str
    lat: float | None
    lon: float | None
    alt: float | None
    image_coordinates: dict[str, int]
    target_confidence: float
    number_confidence: float
    number_source: str
    image_path: str | None
    metadata: dict[str, Any]

    @property
    def score(self) -> float:
        return float(self.target_confidence) * max(float(self.number_confidence), 0.01)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class TargetRegistry:
    def __init__(self):
        self.targets_by_number: dict[str, TargetObservation] = {}
        self.unidentified: list[TargetObservation] = []

    def update(
        self,
        detection: dict[str, Any],
        number_result: dict[str, Any],
        location: dict[str, Any],
        image_path: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> TargetObservation:
        number = number_result.get("number")
        if number is None or str(number).strip() == "":
            number = f"unknown_{len(self.unidentified) + 1}"
            store_as_unknown = True
        else:
            number = str(number)
            store_as_unknown = False

        observation = TargetObservation(
            number=number,
            lat=location.get("lat"),
            lon=location.get("lon"),
            alt=location.get("alt"),
            image_coordinates={
                "ctrX": int(detection["ctrX"]),
                "ctrY": int(detection["ctrY"]),
                "width": int(detection["width"]),
                "height": int(detection["height"]),
            },
            target_confidence=float(detection.get("confidence", 0.0)),
            number_confidence=float(number_result.get("confidence", 0.0)),
            number_source=str(number_result.get("source", "unknown")),
            image_path=image_path,
            metadata=metadata or {},
        )

        if store_as_unknown:
            self.unidentified.append(observation)
            return observation

        previous = self.targets_by_number.get(number)
        if previous is None or observation.score > previous.score:
            self.targets_by_number[number] = observation

        return self.targets_by_number[number]

    def as_list(self) -> list[dict[str, Any]]:
        return [obs.to_dict() for obs in self.targets_by_number.values()]

    def unknown_as_list(self) -> list[dict[str, Any]]:
        return [obs.to_dict() for obs in self.unidentified]
