from __future__ import annotations
import math
from typing import NamedTuple

import pyproj

from geo_utils import WaypointStruct, metric_dist


"""
TSP Solver — 2-opt improvement over nearest-neighbor heuristic.

Points are real-world geodetic coordinates stored as WaypointStruct
(lat/lon in degE7, alt in mm). All distances are in mm throughout.
"""


# ------------
# Data types
# ------------

class Point(NamedTuple):
    """
    WaypointStruct wrapper for use in the TSP solver.
    lat_e7 / lon_e7 are integer degrees * 1e7 (degE7).
    alt_mm is altitude in millimetres.
    """
    lat_e7: int
    lon_e7: int
    alt_mm: float = 0.0
    label: str = ""

    def to_waypoint(self) -> WaypointStruct:
        return WaypointStruct(self.lat_e7, self.lon_e7, self.alt_mm)

    def __repr__(self) -> str:
        return self.label or f"({self.lat_e7 * 1e-7:.6f}, {self.lon_e7 * 1e-7:.6f}, alt={self.alt_mm:.0f}mm)"


class Edge(NamedTuple):
    a: int          # index of start point
    b: int          # index of end point
    weight: float   # geodetic 3D distance in mm


class Direction(NamedTuple):
    step: int
    from_point: Point
    to_point: Point
    distance: float             # 3D distance in mm
    horiz_dist: float           # horizontal geodetic distance in mm
    alt_change: float           # mm delta (positive = ascending, negative = descending)
    angle_of_elevation: float   # degrees
    bearing_deg: float
    compass: str


COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

_GEOD = pyproj.Geod(ellps="WGS84")

_M_TO_MM = 1000.0


# ------------
# TSP Solver
# ------------

class TSP:

    # ------------------
    # Geometry helpers
    # ------------------

    def _horizontal_dist(self, a: Point, b: Point) -> float:
        """
        Horizontal geodetic distance in mm (ignores altitude).
        Zeroes out altitude on both ends so metric_dist measures horizontal only.
        """
        wa = WaypointStruct(a.lat_e7, a.lon_e7, 0.0)
        wb = WaypointStruct(b.lat_e7, b.lon_e7, 0.0)
        return wa.distance_to(wb) * _M_TO_MM

    def _dist_3d(self, a: Point, b: Point) -> float:
        """
        Full 3D distance in mm: geodetic horizontal + altitude delta.
        Delegates to WaypointStruct.distance_to (metric_dist), converts m -> mm.
        """
        return a.to_waypoint().distance_to(b.to_waypoint()) * _M_TO_MM

    def _angle_of_elevation(self, a: Point, b: Point) -> float:
        """
        Angle of elevation in degrees from a to b.
        atan2(vertical_rise_mm, horizontal_dist_mm) — units cancel so no conversion needed.
        """
        horiz = self._horizontal_dist(a, b)
        vert  = b.alt_mm - a.alt_mm     # mm delta
        if horiz == 0.0:
            return 90.0 if vert > 0 else (-90.0 if vert < 0 else 0.0)
        return math.degrees(math.atan2(vert, horiz))

    def tour_length(self, points: list[Point], tour: list[int]) -> float:
        """Total 3D geodetic tour length in mm."""
        n = len(tour)
        return sum(
            self._dist_3d(points[tour[i]], points[tour[(i + 1) % n]])
            for i in range(n)
        )

    def bearing(self, a: Point, b: Point) -> float:
        """
        Forward azimuth in degrees (0 = North, clockwise) from a to b,
        computed by pyproj geodesic inverse.
        """
        lat1, lon1, _ = a.to_waypoint().to_deg()
        lat2, lon2, _ = b.to_waypoint().to_deg()
        az12, _, _ = _GEOD.inv(lon1, lat1, lon2, lat2)
        return az12 % 360

    def compass_direction(self, deg: float) -> str:
        idx = round(deg / 45) % 8
        return COMPASS[idx]

    # ------------------
    # Nearest-neighbour
    # ------------------

    def nearest_neighbor(self, points: list[Point], start: int = 0) -> list[int]:
        """Greedy nearest-neighbour tour starting at `start`. Uses 3D geodetic distance."""
        n = len(points)
        visited = [False] * n
        tour = [start]
        visited[start] = True

        for _ in range(n - 1):
            current = tour[-1]
            best_dist = math.inf
            best_next = -1
            for j in range(n):
                if not visited[j]:
                    d = self._dist_3d(points[current], points[j])
                    if d < best_dist:
                        best_dist = d
                        best_next = j
            tour.append(best_next)
            visited[best_next] = True

        return tour

    # ------------------
    # 2-opt improvement
    # ------------------

    def two_opt(self, points: list[Point], tour: list[int]) -> list[int]:
        """
        Improve a tour using the 2-opt algorithm (3D geodetic distances).

        Repeatedly scans all pairs of edges (i, i+1) and (j, j+1).
        If reversing the segment between them reduces total distance, do it.
        Continues until no improving swap exists.
        """
        best = tour[:]
        improved = True

        while improved:
            improved = False
            n = len(best)

            for i in range(n - 1):
                for j in range(i + 2, n):
                    if i == 0 and j == n - 1:
                        continue

                    a, b = best[i], best[i + 1]
                    c, d = best[j], best[(j + 1) % n]

                    current  = self._dist_3d(points[a], points[b]) + self._dist_3d(points[c], points[d])
                    proposed = self._dist_3d(points[a], points[c]) + self._dist_3d(points[b], points[d])

                    if proposed < current - 1e-3:     # tolerance in mm
                        best[i + 1: j + 1] = best[i + 1: j + 1][::-1]
                        improved = True

        return best

    # ---------------------
    # Directions generator
    # ---------------------

    def build_directions(self, points: list[Point], tour: list[int]) -> list[Direction]:
        directions = []
        n = len(tour)
        for step in range(n):
            a_idx = tour[step]
            b_idx = tour[(step + 1) % n]
            a, b  = points[a_idx], points[b_idx]

            deg = self.bearing(a, b)
            directions.append(Direction(
                step=step + 1,
                from_point=a,
                to_point=b,
                distance=self._dist_3d(a, b),
                horiz_dist=self._horizontal_dist(a, b),
                alt_change=b.alt_mm - a.alt_mm,
                angle_of_elevation=self._angle_of_elevation(a, b),
                bearing_deg=deg,
                compass=self.compass_direction(deg),
            ))
        return directions

    def print_directions(self, directions: list[Direction], total: float) -> None:
        print("\n── Turn-by-turn directions ──────────────────────────────────────────────")
        for d in directions:
            label = "Return to start" if d.step == len(directions) else f"Go to {d.to_point}"
            alt_str = (
                f"↑+{d.alt_change:.0f}mm" if d.alt_change > 0
                else f"↓{d.alt_change:.0f}mm" if d.alt_change < 0
                else "→ flat"
            )
            print(
                f"  Step {d.step:>2}: {str(d.from_point):<30} → {str(d.to_point):<30} "
                f"| {d.compass:<2} ({d.bearing_deg:>6.1f}°) "
                f"| 3D {d.distance:>12.1f} mm "
                f"| horiz {d.horiz_dist:>12.1f} mm "
                f"| elev {d.angle_of_elevation:>+6.2f}° "
                f"| alt {alt_str:<14}"
                f"  ({label})"
            )
        print(f"\n  Total 3D tour distance: {total:.1f} mm\n")

    # -------
    # Solver
    # -------

    def solve_tsp(
        self,
        points: list[Point],
        start: int = 0,
        verbose: bool = False,
    ) -> tuple[list[int], list[Direction], float]:
        """
        Solve TSP using nearest-neighbour + 2-opt (full 3D geodetic distances).

        Args:
            points  — list of Point (lat_e7, lon_e7, alt_mm, label)
            start   — index of the starting point
            verbose — print progress and turn-by-turn directions

        Returns:
            tour       — ordered list of point indices
            directions — step-by-step Direction objects
            total_dist — total 3D geodetic tour length in mm
        """
        if len(points) < 2:
            raise ValueError("Need at least 2 points.")

        if verbose:
            print(f"\nSolving TSP for {len(points)} points  (start index = {start})")

        nn_tour = self.nearest_neighbor(points, start=start)
        nn_len  = self.tour_length(points, nn_tour)
        if verbose:
            print(f"  Nearest-neighbour length : {nn_len:.1f} mm")

        opt_tour = self.two_opt(points, nn_tour)
        opt_len  = self.tour_length(points, opt_tour)
        if verbose:
            improvement = (nn_len - opt_len) / nn_len * 100 if nn_len else 0
            print(f"  2-opt improved length    : {opt_len:.1f} mm  ({improvement:.1f}% better)")
            print(f"  Tour order               : {' → '.join(str(points[i]) for i in opt_tour)} → start")

        directions = self.build_directions(points, opt_tour)

        if verbose:
            self.print_directions(directions, opt_len)

        return opt_tour, directions, opt_len


# -------
# Demo
# -------

if __name__ == "__main__":
    # degE7 = degrees * 1e7,  alt in mm
    demo_points = [
        Point(337654320,  -1178234560,      0, "A"),
        Point(337660000,  -1178220000, 50_000, "B"),
        Point(337670000,  -1178210000, 50_000, "C"),
        Point(337665000,  -1178230000, 50_000, "D"),
        Point(337655000,  -1178240000,      0, "E"),
        Point(337648000,  -1178225000, 50_000, "F"),
        Point(337662000,  -1178215000, 50_000, "G"),
        Point(337650000,  -1178235000, 50_000, "H"),
    ]

    tsp = TSP()
    tour, directions, total = tsp.solve_tsp(demo_points, start=0, verbose=True)

    print("── Programmatic access ─────────────────────────────────────")
    print(f"Tour indices : {tour}")
    print(f"Total dist   : {total:.1f} mm")
    print(f"First leg    : {directions[0].from_point} → {directions[0].to_point} "
          f"({directions[0].compass}, 3D dist {directions[0].distance:.1f} mm, "
          f"alt change {directions[0].alt_change:+.0f} mm)")
