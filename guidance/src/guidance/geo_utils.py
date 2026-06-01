import math
from typing import Self


class WaypointStruct:
    """
    Waypoint class for convenience.
    If lat/lon are ints, they are stored as degE7,
    Alt is always in mm.
    """

    lat: int = 0
    lon: int = 0
    alt: float = 0.0

    def __init__(self, lat_e7: int, lon_e7: int, alt_mm: float):
        """
        Waypoint class for convenience.
        If lat/lon are ints, they are degE7,
        Alt is always in mm.
        """
        self.lat = lat_e7
        self.lon = lon_e7
        self.alt = alt_mm

    def to_deg(self) -> tuple[float, float, float]:
        """
        Waypoint class for convenience.
        returns lat/lon in deg, alt in mm
        """
        return (self.lat * 1e-7, self.lon * 1e-7, self.alt)

    def distance_to(self, other: Self) -> float:
        """
        Distance in (m) to other
        """
        return metric_dist(self, other)

    def __str__(self):
        return f"{self.lat}, {self.lon}, {self.alt}"


def metric_dist(w1: WaypointStruct, w2: WaypointStruct) -> float:
    lat1, lon1, alt1 = w1.to_deg()
    lat2, lon2, alt2 = w2.to_deg()

    R = 6_371_000.0  # Earth radius in metres
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)

    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    )
    distance_m = R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    dalt_m = (alt2 - alt1) / 1000.0
    return round(math.sqrt(distance_m**2 + dalt_m**2), 3)
