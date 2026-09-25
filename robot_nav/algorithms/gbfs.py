"""
Greedy Best-First Search (GBFS) Algorithm for Grid Navigation.
Informed heuristic search expanding nodes strictly minimizing distance estimate to goal.
Author: Abdul Rehman Rattu
"""

from typing import List, Tuple, Dict, Set
import heapq
import time
from robot_nav.algorithms.base import BaseSearcher, SearchResult
from robot_nav.heuristics import multi_goal_heuristic


class GreedyBestFirstSearch(BaseSearcher):
    """
    Greedy Best-First Search (GBFS).
    Evaluation Function: f(n) = h(n).
    Fast convergence, but not guaranteed to find cost-optimal paths.
    """

    def __init__(self, grid_map, metric: str = "manhattan"):
        super().__init__(grid_map)
        self.metric = metric

    def search(self) -> SearchResult:
        start_time = time.perf_counter()
        start = self.grid.start
        goals = self.grid.goals

        if self.grid.is_goal(start[0], start[1]):
            return SearchResult(
                algorithm="GBFS",
                success=True,
                path=[start],
                actions=[],
                nodes_expanded=1,
                path_cost=0,
                duration_ms=(time.perf_counter() - start_time) * 1000,
                visited_nodes=[start],
            )

        counter = 0
        h_start = multi_goal_heuristic(start, goals, self.metric)
        frontier = [(h_start, counter, start)]

        visited: Set[Tuple[int, int]] = {start}
        parent_map: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited_order: List[Tuple[int, int]] = []
        nodes_expanded = 0

        while frontier:
            _, _, current = heapq.heappop(frontier)
            nodes_expanded += 1
            visited_order.append(current)

            if self.grid.is_goal(current[0], current[1]):
                duration_ms = (time.perf_counter() - start_time) * 1000
                path, actions = self.reconstruct_path(parent_map, current, start)
                return SearchResult(
                    algorithm="GBFS",
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
                    counter += 1
                    h_val = multi_goal_heuristic(neighbor, goals, self.metric)
                    heapq.heappush(frontier, (h_val, counter, neighbor))

        duration_ms = (time.perf_counter() - start_time) * 1000
        return SearchResult(
            algorithm="GBFS",
            success=False,
            nodes_expanded=nodes_expanded,
            duration_ms=duration_ms,
            visited_nodes=visited_order,
        )
