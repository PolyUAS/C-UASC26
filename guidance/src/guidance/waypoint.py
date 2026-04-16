from __future__ import annotations
import math
from typing import NamedTuple

def lat_long_to_euclidian(lat: int, long: int) -> Point:
    """Convert to euclidian points from latitude and longitude"""
    x = math.cos(lat) * math.cos(long)
    y = math.cos(lat) * math.sin(long)
    return Point(x, y)


"""
TSP Solver — 2-opt improvement over nearest-neighbor heuristic.

Points are Euclidean (x, y) tuples. Edge weights are Euclidean distances.
The solver returns an ordered list of point indices representing the tour,
plus step-by-step directions for each leg.
"""

# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

class Point(NamedTuple):
    x: float
    y: float
    label: str = ""

    def __repr__(self) -> str:
        name = self.label or f"({self.x}, {self.y})"
        return name


class Edge(NamedTuple):
    a: int          # index of start point
    b: int          # index of end point
    weight: float   # Euclidean distance


class Direction(NamedTuple):
    step: int
    from_point: Point
    to_point: Point
    distance: float
    bearing_deg: float 
    compass: str


# ---------------------------------------------------------------------------
# Core geometry
# ---------------------------------------------------------------------------

def euclidean(a: Point, b: Point) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def tour_length(points: list[Point], tour: list[int]) -> float:
    n = len(tour)
    return sum(
        euclidean(points[tour[i]], points[tour[(i + 1) % n]])
        for i in range(n)
    )


def bearing(a: Point, b: Point) -> float:
    """Angle in degrees from a to b, measured clockwise from north (0 = up)."""
    dx = b.x - a.x
    dy = b.y - a.y                  # y increases downward in screen coords
    deg = math.degrees(math.atan2(dx, -dy))  # clockwise from north
    return deg % 360


COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

def compass_direction(deg: float) -> str:
    idx = round(deg / 45) % 8
    return COMPASS[idx]


# ---------------------------------------------------------------------------
# Nearest-neighbor heuristic (initial tour)
# ---------------------------------------------------------------------------

def nearest_neighbor(points: list[Point], start: int = 0) -> list[int]:
    """Greedy nearest-neighbor tour starting at `start`."""
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
                d = euclidean(points[current], points[j])
                if d < best_dist:
                    best_dist = d
                    best_next = j
        tour.append(best_next)
        visited[best_next] = True

    return tour


# ---------------------------------------------------------------------------
# 2-opt improvement
# ---------------------------------------------------------------------------

def two_opt(points: list[Point], tour: list[int]) -> list[int]:
    """
    Improve a tour using the 2-opt algorithm.

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
                # Skip the wrap-around edge when i == 0
                if i == 0 and j == n - 1:
                    continue

                a, b = best[i], best[i + 1]
                c, d = best[j], best[(j + 1) % n]

                # Current cost: a→b and c→d
                # Proposed cost: a→c and b→d  (reverse segment i+1..j)
                current = euclidean(points[a], points[b]) + euclidean(points[c], points[d])
                proposed = euclidean(points[a], points[c]) + euclidean(points[b], points[d])

                if proposed < current - 1e-10:
                    # Reverse the segment between i+1 and j (inclusive)
                    best[i + 1 : j + 1] = best[i + 1 : j + 1][::-1]
                    improved = True

    return best


# ---------------------------------------------------------------------------
# Directions generator
# ---------------------------------------------------------------------------

def build_directions(points: list[Point], tour: list[int]) -> list[Direction]:
    directions = []
    n = len(tour)
    for step in range(n):
        a_idx = tour[step]
        b_idx = tour[(step + 1) % n]
        a, b = points[a_idx], points[b_idx]
        d = euclidean(a, b)
        deg = bearing(a, b)
        directions.append(Direction(
            step=step + 1,
            from_point=a,
            to_point=b,
            distance=d,
            bearing_deg=deg,
            compass=compass_direction(deg),
        ))
    return directions


def print_directions(directions: list[Direction], total: float) -> None:
    print("\n── Turn-by-turn directions ─────────────────────────────────")
    for d in directions:
        label = "Return to start" if d.step == len(directions) else f"Go to {d.to_point}"
        print(
            f"  Step {d.step:>2}: {str(d.from_point):<20} → {str(d.to_point):<20} "
            f"| {d.compass:<2} ({d.bearing_deg:>6.1f}°) "
            f"| dist {d.distance:>8.2f}"
            f"  ({label})"
        )
    print(f"\n  Total tour distance: {total:.2f}\n")


# ---------------------------------------------------------------------------
# High-level solver
# ---------------------------------------------------------------------------

def solve_tsp(
    points: list[Point],
    start: int = 0,
    verbose: bool = False,
) -> tuple[list[int], list[Direction], float]:
    """
    Solve TSP using nearest-neighbor + 2-opt.

    Returns:
        tour       — ordered list of point indices
        directions — step-by-step Direction objects
        total_dist — total tour length
    """
    if len(points) < 2:
        raise ValueError("Need at least 2 points.")

    if verbose:
        print(f"\nSolving TSP for {len(points)} points  (start index = {start})")

    # Phase 1: nearest-neighbor initial tour
    nn_tour = nearest_neighbor(points, start=start)
    nn_len = tour_length(points, nn_tour)
    if verbose:
        print(f"  Nearest-neighbor length : {nn_len:.2f}")

    # Phase 2: 2-opt improvement
    opt_tour = two_opt(points, nn_tour)
    opt_len = tour_length(points, opt_tour)
    if verbose:
        improvement = (nn_len - opt_len) / nn_len * 100
        print(f"  2-opt improved length   : {opt_len:.2f}  ({improvement:.1f}% better)")
        print(f"  Tour order              : {' → '.join(str(points[i]) for i in opt_tour)} → start")

    directions = build_directions(points, opt_tour)

    if verbose:
        print_directions(directions, opt_len)

    return opt_tour, directions, opt_len





# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------


if __name__ == "__main__":
    demo_points = [
            Point(100, 200, "A"),
            Point(300, 100, "B"),
            Point(500, 250, "C"),
            Point(450, 450, "D"),
            Point(250, 400, "E"),
            Point(150, 320, "F"),
            Point(370, 300, "G"),
            Point(200, 150, "H"),
        ]

    tour, directions, total = solve_tsp(demo_points, start=0, verbose=True)

    # You can also use the results programmatically:
    print("── Programmatic access ─────────────────────────────────────")
    print(f"Tour indices : {tour}")
    print(f"Total dist   : {total:.2f}")
    print(f"First leg    : {directions[0].from_point} → {directions[0].to_point} "
            f"({directions[0].compass}, {directions[0].distance:.2f} units)")

    print(directions)
