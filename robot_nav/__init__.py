"""
Autonomous Robot Grid Navigation & Heuristic Search Suite.
Core package providing discrete grid environments, heuristic distance metrics,
and graph traversal algorithms.
Author: Abdul Rehman Rattu
"""

from robot_nav.grid import GridMap
from robot_nav.heuristics import (
    manhattan_distance,
    euclidean_distance,
    chebyshev_distance,
    multi_goal_heuristic,
)
from robot_nav.algorithms import (
    SearchResult,
    BreadthFirstSearch,
    DepthFirstSearch,
    GreedyBestFirstSearch,
    AStarSearch,
    DFSBFSHybridSearch,
    BidirectionalAStarSearch,
)

__version__ = "1.0.0"
__author__ = "Abdul Rehman Rattu"

__all__ = [
    "GridMap",
    "manhattan_distance",
    "euclidean_distance",
    "chebyshev_distance",
    "multi_goal_heuristic",
    "SearchResult",
    "BreadthFirstSearch",
    "DepthFirstSearch",
    "GreedyBestFirstSearch",
    "AStarSearch",
    "DFSBFSHybridSearch",
    "BidirectionalAStarSearch",
]
