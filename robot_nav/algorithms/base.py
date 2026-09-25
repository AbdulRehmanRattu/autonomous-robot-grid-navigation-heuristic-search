"""
Abstract Base Searcher and Search Result Dataclass.
Standardizes node tracking, timing, step orders, and path reconstruction.
Author: Abdul Rehman Rattu
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional, Any
from abc import ABC, abstractmethod
import time
from robot_nav.grid import GridMap


@dataclass
class SearchResult:
    """Standardized output telemetry for all graph search algorithms."""
    algorithm: str
    success: bool
    path: List[Tuple[int, int]] = field(default_factory=list)
    actions: List[str] = field(default_factory=list)
    nodes_expanded: int = 0
    path_cost: int = 0
    duration_ms: float = 0.0
    visited_nodes: List[Tuple[int, int]] = field(default_factory=list)

    def summary(self) -> str:
        status = "SOLVED" if self.success else "NO SOLUTION"
        action_seq = " -> ".join(self.actions) if self.actions else "None"
        return (
            f"[{self.algorithm}] Status: {status} | Nodes Expanded: {self.nodes_expanded} | "
            f"Path Cost: {self.path_cost} steps | Time: {self.duration_ms:.2f} ms\n"
            f"Actions: {action_seq}"
        )


class BaseSearcher(ABC):
    """Abstract base class for all grid search implementations."""

    def __init__(self, grid_map: GridMap):
        self.grid = grid_map
        # Standardized movement order: Up, Left, Down, Right
        self.direction_order = ("up", "left", "down", "right")
        self.offsets = {
            "up": (0, -1),
            "left": (-1, 0),
            "down": (0, 1),
            "right": (1, 0),
        }
        self.inverse_offsets = {v: k for k, v in self.offsets.items()}

    @abstractmethod
    def search(self) -> SearchResult:
        """Execute graph search and return SearchResult."""
        pass

    def reconstruct_path(
        self,
        parent_map: Dict[Tuple[int, int], Tuple[int, int]],
        current: Tuple[int, int],
        start: Tuple[int, int],
    ) -> Tuple[List[Tuple[int, int]], List[str]]:
        """Backtrack from goal to start to construct forward path and action string."""
        path = [current]
        curr = current
        while curr != start and curr in parent_map:
            curr = parent_map[curr]
            path.append(curr)
        path.reverse()

        actions = []
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            act = self.inverse_offsets.get((dx, dy), "unknown")
            actions.append(act)

        return path, actions
