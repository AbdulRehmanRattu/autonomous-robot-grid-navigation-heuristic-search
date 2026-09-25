"""
Breadth-First Search (BFS) Algorithm for Grid Navigation.
Uninformed search exploring nodes layer-by-layer; guarantees shortest path on unit-cost graphs.
Author: Abdul Rehman Rattu
"""

from typing import List, Tuple, Dict, Set
from collections import deque
import time
from robot_nav.algorithms.base import BaseSearcher, SearchResult


class BreadthFirstSearch(BaseSearcher):
    """
    Breadth-First Search (BFS) Implementation.
    Uses FIFO queue for frontier management.
    Time Complexity: O(b^d), Space Complexity: O(b^d).
    """

    def search(self) -> SearchResult:
        start_time = time.perf_counter()
        start = self.grid.start

        if self.grid.is_goal(start[0], start[1]):
            return SearchResult(
                algorithm="BFS",
                success=True,
                path=[start],
                actions=[],
                nodes_expanded=1,
                path_cost=0,
                duration_ms=(time.perf_counter() - start_time) * 1000,
                visited_nodes=[start],
            )

        queue = deque([start])
        visited: Set[Tuple[int, int]] = {start}
        parent_map: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited_order: List[Tuple[int, int]] = []
        nodes_expanded = 0

        while queue:
            current = queue.popleft()
            nodes_expanded += 1
            visited_order.append(current)

            if self.grid.is_goal(current[0], current[1]):
                duration_ms = (time.perf_counter() - start_time) * 1000
                path, actions = self.reconstruct_path(parent_map, current, start)
                return SearchResult(
                    algorithm="BFS",
                    success=True,
                    path=path,
                    actions=actions,
                    nodes_expanded=nodes_expanded,
                    path_cost=len(path) - 1,
                    duration_ms=duration_ms,
                    visited_nodes=visited_order,
                )

            for act, neighbor in self.grid.get_neighbors(current[0], current[1], self.direction_order):
                if neighbor not in visited:
                    visited.add(neighbor)
                    parent_map[neighbor] = current
                    queue.append(neighbor)

        duration_ms = (time.perf_counter() - start_time) * 1000
        return SearchResult(
            algorithm="BFS",
            success=False,
            nodes_expanded=nodes_expanded,
            duration_ms=duration_ms,
            visited_nodes=visited_order,
        )
