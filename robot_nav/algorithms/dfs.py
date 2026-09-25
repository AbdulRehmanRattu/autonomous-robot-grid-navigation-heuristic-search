"""
Depth-First Search (DFS) Algorithm for Grid Navigation.
Uninformed search exploring deep paths via LIFO stack frontier.
Author: Abdul Rehman Rattu
"""

from typing import List, Tuple, Dict, Set
import time
from robot_nav.algorithms.base import BaseSearcher, SearchResult


class DepthFirstSearch(BaseSearcher):
    """
    Depth-First Search (DFS) Implementation.
    Uses LIFO stack for frontier management.
    Time Complexity: O(b^m), Space Complexity: O(b * m).
    """

    def search(self) -> SearchResult:
        start_time = time.perf_counter()
        start = self.grid.start

        if self.grid.is_goal(start[0], start[1]):
            return SearchResult(
                algorithm="DFS",
                success=True,
                path=[start],
                actions=[],
                nodes_expanded=1,
                path_cost=0,
                duration_ms=(time.perf_counter() - start_time) * 1000,
                visited_nodes=[start],
            )

        stack: List[Tuple[int, int]] = [start]
        visited: Set[Tuple[int, int]] = set()
        parent_map: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited_order: List[Tuple[int, int]] = []
        nodes_expanded = 0

        # Note: In standard DFS stack, we reverse neighbor pushing to explore
        # ('up', 'left', 'down', 'right') in correct priority
        reversed_order = tuple(reversed(self.direction_order))

        while stack:
            current = stack.pop()

            if current in visited:
                continue

            visited.add(current)
            nodes_expanded += 1
            visited_order.append(current)

            if self.grid.is_goal(current[0], current[1]):
                duration_ms = (time.perf_counter() - start_time) * 1000
                path, actions = self.reconstruct_path(parent_map, current, start)
                return SearchResult(
                    algorithm="DFS",
                    success=True,
                    path=path,
                    actions=actions,
                    nodes_expanded=nodes_expanded,
                    path_cost=len(path) - 1,
                    duration_ms=duration_ms,
                    visited_nodes=visited_order,
                )

            for act, neighbor in self.grid.get_neighbors(current[0], current[1], reversed_order):
                if neighbor not in visited:
                    parent_map[neighbor] = current
                    stack.append(neighbor)

        duration_ms = (time.perf_counter() - start_time) * 1000
        return SearchResult(
            algorithm="DFS",
            success=False,
            nodes_expanded=nodes_expanded,
            duration_ms=duration_ms,
            visited_nodes=visited_order,
        )
