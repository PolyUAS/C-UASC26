import math
from typing import Self

import pyproj


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


def metric_dist(w1: WaypointStruct, w2: WaypointStruct) -> float:
    """
    Given coord params, returns distance in meters btw/ pts.
    """

    lat1, lon1, alt1 = w1.to_deg()
    lat2, lon2, alt2 = w2.to_deg()

    dalt_m = (alt2 - alt1) / 1000.0  # mm to m

    geod = pyproj.Geod(ellps="WGS84")

    az12, az21, distance_m = geod.inv(lon1, lat1, lon2, lat2)

    return math.sqrt(distance_m**2 + dalt_m**2)
