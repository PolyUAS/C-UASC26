from __future__ import annotations
import math
from typing import NamedTuple


"""
TSP Solver — 2-opt improvement over nearest-neighbor heuristic.

Points are Euclidean (x, y, z). Edge weights use full 3D Euclidean distance.
The solver returns an ordered list of point indices representing the tour,
plus step-by-step directions for each leg (including altitude change).
"""
"""
Currently works by inputing a list of points.
"""
# ------------
# Data types
# ------------

class Point(NamedTuple):
    x: float
    y: float
    z: float = 0.0
    label: str = ""

    def __repr__(self) -> str:
        name = self.label or f"({self.x}, {self.y}, {self.z})"
        return name


class Edge(NamedTuple):
    a: int          # index of start point
    b: int          # index of end point
    weight: float   # 3D Euclidean distance


class Direction(NamedTuple):
    step: int
    from_point: Point
    to_point: Point
    distance: float     # 3D Euclidean distance (horizontal + vertical)
    horiz_dist: float   # horizontal (x, y) distance only
    alt_change: float   # z_to - z_from (positive = ascending, negative = descending)
    bearing_deg: float
    compass: str


COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


class TSP(object):


    # Geometry


    def lat_lon_to_euclidean(self, lat: float, lon: float, alt: float) -> Point:
        """Convert latitude, longitude, altitude to Euclidean (x, y, z) point."""
        x = math.cos(math.radians(lat)) * math.cos(math.radians(lon))
        y = math.cos(math.radians(lat)) * math.sin(math.radians(lon))
        z = alt * math.cos(math.radians(lat))
        return Point(x, y, z)

    def euclidean_2d(self, a: Point, b: Point) -> float:
        """Horizontal-only distance (ignores z). Used for bearing calculations."""
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def euclidean_3d(self, a: Point, b: Point) -> float:
        """Full 3D Euclidean distance including altitude (z)."""
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def tour_length(self, points: list[Point], tour: list[int]) -> float:
        """Total 3D tour length."""
        n = len(tour)
        return sum(
            self.euclidean_3d(points[tour[i]], points[tour[(i + 1) % n]])
            for i in range(n)
        )

    def bearing(self, a: Point, b: Point) -> float:
        """Angle in degrees from a to b, measured clockwise from north (0 = up)."""
        dx = b.x - a.x
        dy = b.y - a.y
        deg = math.degrees(math.atan2(dx, -dy))  # clockwise from north
        return deg % 360

    def compass_direction(self, deg: float) -> str:
        idx = round(deg / 45) % 8
        return COMPASS[idx]

    def nearest_neighbor(self, points: list[Point], start: int = 0) -> list[int]:
        """Greedy nearest-neighbor tour starting at `start`. Uses 3D distance."""
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
                    d = self.euclidean_3d(points[current], points[j])
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
        Improve a tour using the 2-opt algorithm (uses 3D distances).

        Repeatedly scans all pairs of edges (i, i+1) and (j, j+1).
        If reversing the segment between them reduces total distance, do it.
        Continues until no improving swap exists.

        Time complexity: O(n²) per pass, O(n² * passes) overall.
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

                    current  = self.euclidean_3d(points[a], points[b]) + self.euclidean_3d(points[c], points[d])
                    proposed = self.euclidean_3d(points[a], points[c]) + self.euclidean_3d(points[b], points[d])

                    if proposed < current - 1e-10:
                        best[i + 1 : j + 1] = best[i + 1 : j + 1][::-1]
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
            a, b = points[a_idx], points[b_idx]

            dist_3d   = self.euclidean_3d(a, b)
            dist_2d   = self.euclidean_2d(a, b)
            alt_change = b.z - a.z
            deg        = self.bearing(a, b)

            directions.append(Direction(
                step=step + 1,
                from_point=a,
                to_point=b,
                distance=dist_3d,
                horiz_dist=dist_2d,
                alt_change=alt_change,
                bearing_deg=deg,
                compass=self.compass_direction(deg),
            ))
        return directions

    def print_directions(self, directions: list[Direction], total: float) -> None:
        print("\n── Turn-by-turn directions ──────────────────────────────────────────────")
        for d in directions:
            label = "Return to start" if d.step == len(directions) else f"Go to {d.to_point}"
            alt_str = (
                f"↑+{d.alt_change:.1f}" if d.alt_change > 0
                else f"↓{d.alt_change:.1f}" if d.alt_change < 0
                else "→ flat"
            )
            print(
                f"  Step {d.step:>2}: {str(d.from_point):<20} → {str(d.to_point):<20} "
                f"| {d.compass:<2} ({d.bearing_deg:>6.1f}°) "
                f"| 3D dist {d.distance:>8.2f} "
                f"| horiz {d.horiz_dist:>8.2f} "
                f"| alt {alt_str:<10}"
                f"  ({label})"
            )
        print(f"\n  Total 3D tour distance: {total:.2f}\n")

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
        Solve TSP using nearest-neighbor + 2-opt (full 3D distances).

        Returns:
            tour       — ordered list of point indices
            directions — step-by-step Direction objects
            total_dist — total 3D tour length
        """
        if len(points) < 2:
            raise ValueError("Need at least 2 points.")

        if verbose:
            print(f"\nSolving TSP for {len(points)} points  (start index = {start})")

        nn_tour = self.nearest_neighbor(points, start=start)
        nn_len  = self.tour_length(points, nn_tour)
        if verbose:
            print(f"  Nearest-neighbor length : {nn_len:.2f}")

        opt_tour = self.two_opt(points, nn_tour)
        opt_len  = self.tour_length(points, opt_tour)
        if verbose:
            improvement = (nn_len - opt_len) / nn_len * 100 if nn_len else 0
            print(f"  2-opt improved length   : {opt_len:.2f}  ({improvement:.1f}% better)")
            print(f"  Tour order              : {' → '.join(str(points[i]) for i in opt_tour)} → start")

        directions = self.build_directions(points, opt_tour)

        if verbose:
            self.print_directions(directions, opt_len)

        return opt_tour, directions, opt_len


# -------
# Demo
# -------

if __name__ == "__main__":
    demo_points = [
        Point(100, 200, 0,   "A"),
        Point(300, 100, 50,  "B"),
        Point(500, 250, 50,  "C"),
        Point(450, 450, 50, "D"),
        Point(250, 400, 0,  "E"),
        Point(150, 320, 50,  "F"),
        Point(370, 300, 50,  "G"),
        Point(200, 150, 50,  "H"),
    ]

    tsp = TSP()
    tour, directions, total = tsp.solve_tsp(demo_points, start=0, verbose=True)

    print("── Programmatic access ─────────────────────────────────────")
    print(f"Tour indices : {tour}")
    print(f"Total dist   : {total:.2f}")
    print(f"First leg    : {directions[0].from_point} → {directions[0].to_point} "
          f"({directions[0].compass}, 3D dist {directions[0].distance:.2f}, "
          f"alt change {directions[0].alt_change:+.1f})")
