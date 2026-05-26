import math

from guidance.geo_utils import WaypointStruct


def _coord_to_deg(point) -> tuple[float, float]:
    if hasattr(point, "Lat") and hasattr(point, "Lon"):
        return float(point.Lat()), float(point.Lon())
    return float(point.lat) / 10_000_000.0, float(point.lon) / 10_000_000.0


def _deg_to_local_xy(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """
    Converts lat/lon degrees to local x/y meters.

    x = east/west meters
    y = north/south meters
    """
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = 111_320.0 * math.cos(math.radians(origin_lat))

    x = (lon - origin_lon) * meters_per_deg_lon
    y = (lat - origin_lat) * meters_per_deg_lat

    return x, y


def _local_xy_to_deg(
    x: float,
    y: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """
    Converts local x/y meters back to lat/lon degrees.
    """
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = 111_320.0 * math.cos(math.radians(origin_lat))

    lat = origin_lat + (y / meters_per_deg_lat)
    lon = origin_lon + (x / meters_per_deg_lon)

    return lat, lon


def _point_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    """
    Distance from point P to line segment AB, in meters.
    """
    dx = bx - ax
    dy = by - ay

    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))

    closest_x = ax + t * dx
    closest_y = ay + t * dy

    return math.hypot(px - closest_x, py - closest_y)


def _distance_to_polygon_boundary(
    x: float,
    y: float,
    polygon: list[tuple[float, float]],
) -> float:
    """
    Shortest distance from a point to any polygon edge.
    """
    best = math.inf

    for i in range(len(polygon)):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % len(polygon)]

        distance = _point_segment_distance(x, y, ax, ay, bx, by)
        best = min(best, distance)

    return best


def _horizontal_intersections(
    polygon: list[tuple[float, float]],
    y: float,
) -> list[float]:
    """
    Finds all x values where a horizontal sweep row crosses the polygon.

    Every pair of returned x values is an inside-polygon segment.
    Example:
        [x1, x2] means one segment.
        [x1, x2, x3, x4] means two segments.
    """
    xs: list[float] = []

    for i in range(len(polygon)):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % len(polygon)]

        # Skip horizontal edges to avoid duplicate intersections.
        if y1 == y2:
            continue

        # Half-open interval avoids double-counting vertices.
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / (y2 - y1)
            x = x1 + t * (x2 - x1)
            xs.append(x)

    xs.sort()
    return xs


def _safe_subsegments_on_row(
    polygon: list[tuple[float, float]],
    x_start: float,
    x_end: float,
    y: float,
    margin_m: float,
    sample_step_m: float,
) -> list[tuple[float, float]]:
    """
    Takes one inside-polygon row segment and removes parts that are too close
    to the geofence boundary.

    This is what creates the safety buffer.
    """
    if x_end < x_start:
        x_start, x_end = x_end, x_start

    safe_segments: list[tuple[float, float]] = []
    active_start: float | None = None

    x = x_start

    while x <= x_end:
        distance_to_boundary = _distance_to_polygon_boundary(x, y, polygon)
        safe = distance_to_boundary >= margin_m

        if safe and active_start is None:
            active_start = x

        elif not safe and active_start is not None:
            active_end = x - sample_step_m

            if active_end > active_start:
                safe_segments.append((active_start, active_end))

            active_start = None

        x += sample_step_m

    if active_start is not None and x_end > active_start:
        safe_segments.append((active_start, x_end))

    return safe_segments


def generate_lawnmower_sweep(
    geofence,
    alt_mm: float,
    spacing_m: float = 8.0,
    margin_m: float = 8.0,
    sample_step_m: float = 1.0,
) -> list[WaypointStruct]:
    """
    Generates a snake/lawnmower waypoint path inside an irregular geofence.

    It does not follow the outside geofence boundary.
    It tries to keep generated waypoints at least margin_m away from the real
    geofence boundary.

    Args:
        geofence:
            self.geofence from missionFSM.py.
            Each point should support point.Lat() and point.Lon().

        alt_mm:
            Target altitude in millimeters.

        spacing_m:
            Distance between sweep rows.

        margin_m:
            Safety distance from the real geofence boundary.

        sample_step_m:
            Boundary-check resolution in meters.

    Returns:
        Waypoints in snake order.
    """
    if len(geofence) < 3:
        raise ValueError("geofence must contain at least 3 points")

    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be greater than 0")

    if margin_m <= 0.0:
        raise ValueError("margin_m must be greater than 0")

    if sample_step_m <= 0.0:
        raise ValueError("sample_step_m must be greater than 0")

    lat_lon_points = [_coord_to_deg(point) for point in geofence]

    origin_lat = sum(lat for lat, _lon in lat_lon_points) / len(lat_lon_points)
    origin_lon = sum(lon for _lat, lon in lat_lon_points) / len(lat_lon_points)

    polygon = [
        _deg_to_local_xy(
            lat=lat,
            lon=lon,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
        )
        for lat, lon in lat_lon_points
    ]

    min_y = min(y for _x, y in polygon)
    max_y = max(y for _x, y in polygon)

    rows: list[tuple[float, list[tuple[float, float]]]] = []

    y = min_y + margin_m

    while y <= max_y - margin_m:
        intersections = _horizontal_intersections(polygon, y)

        row_segments: list[tuple[float, float]] = []

        # Every pair of intersections is an inside-polygon segment.
        for i in range(0, len(intersections) - 1, 2):
            x_start = intersections[i]
            x_end = intersections[i + 1]

            safe_segments = _safe_subsegments_on_row(
                polygon=polygon,
                x_start=x_start,
                x_end=x_end,
                y=y,
                margin_m=margin_m,
                sample_step_m=sample_step_m,
            )

            row_segments.extend(safe_segments)

        if row_segments:
            rows.append((y, row_segments))

        y += spacing_m

    waypoints: list[WaypointStruct] = []
    left_to_right = True

    for row_y, row_segments in rows:
        row_segments.sort(key=lambda segment: segment[0])

        if not left_to_right:
            row_segments.reverse()

        for x1, x2 in row_segments:
            if left_to_right:
                ordered_points = [(x1, row_y), (x2, row_y)]
            else:
                ordered_points = [(x2, row_y), (x1, row_y)]

            for x, point_y in ordered_points:
                lat, lon = _local_xy_to_deg(
                    x=x,
                    y=point_y,
                    origin_lat=origin_lat,
                    origin_lon=origin_lon,
                )

                waypoints.append(
                    WaypointStruct(
                        round(lat * 1e7),
                        round(lon * 1e7),
                        alt_mm,
                    )
                )

        left_to_right = not left_to_right

    if len(waypoints) < 2:
        raise ValueError(
            "No safe lawnmower waypoints were generated. "
            "Try reducing margin_m or spacing_m."
        )

    return waypoints
