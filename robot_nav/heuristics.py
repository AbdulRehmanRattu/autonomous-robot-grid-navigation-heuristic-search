"""
Heuristic Metric Distance Functions for Grid Search.
Provides admissible heuristics (Manhattan, Euclidean, Chebyshev) for unit-cost 4-way motion.
Author: Abdul Rehman Rattu
"""

from typing import Tuple, List
import math


def manhattan_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
    """
    Taxicab / L1 Metric: |x1 - x2| + |y1 - y2|.
    Admissible and consistent for 4-directional grid motion without diagonal traversal.
    """
    return float(abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]))


def euclidean_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
    """
    Straight-line / L2 Metric: sqrt((x1 - x2)^2 + (y1 - y2)^2).
    Strict lower bound for 4-way motion (strictly admissible).
    """
    return float(math.hypot(p1[0] - p2[0], p1[1] - p2[1]))


def chebyshev_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
    """L_infinity Metric: max(|x1 - x2|, |y1 - y2|)."""
    return float(max(abs(p1[0] - p2[0]), abs(p1[1] - p2[1])))


def multi_goal_heuristic(
    current: Tuple[int, int],
    goals: List[Tuple[int, int]],
    metric: str = "manhattan",
) -> float:
    """
    Calculates minimal distance across all target goals:
        h(n) = min_{g in Goals} d(n, g)
    Preserves admissibility since cost to nearest goal is <= cost to any goal.
    """
    if not goals:
        return 0.0

    fn = manhattan_distance if metric == "manhattan" else euclidean_distance
    return min(fn(current, g) for g in goals)
