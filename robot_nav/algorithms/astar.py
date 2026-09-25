"""
A* Search Algorithm for Grid Navigation.
Informed search minimizing f(n) = g(n) + h(n); guarantees optimal path cost with admissible heuristics.
Author: Abdul Rehman Rattu
"""

from typing import List, Tuple, Dict, Set
import heapq
import time
from robot_nav.algorithms.base import BaseSearcher, SearchResult
from robot_nav.heuristics import multi_goal_heuristic


class AStarSearch(BaseSearcher):
    """
    A* Search Algorithm.
    Evaluation Function: f(n) = g(n) + h(n).
    Optimal and Complete with consistent and admissible heuristics.
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
                algorithm="A*",
                success=True,
                path=[start],
                actions=[],
                nodes_expanded=1,
                path_cost=0,
                duration_ms=(time.perf_counter() - start_time) * 1000,
                visited_nodes=[start],
            )

        counter = 0
        g_score: Dict[Tuple[int, int], float] = {start: 0.0}
        f_start = multi_goal_heuristic(start, goals, self.metric)
        frontier = [(f_start, counter, start)]

        visited: Set[Tuple[int, int]] = set()
        parent_map: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited_order: List[Tuple[int, int]] = []
        nodes_expanded = 0

        while frontier:
            f_val, _, current = heapq.heappop(frontier)

            if current in visited:
                continue

            visited.add(current)
            nodes_expanded += 1
            visited_order.append(current)

            if self.grid.is_goal(current[0], current[1]):
                duration_ms = (time.perf_counter() - start_time) * 1000
                path, actions = self.reconstruct_path(parent_map, current, start)
                return SearchResult(
                    algorithm="A*",
                    success=True,
                    path=path,
                    actions=actions,
                    nodes_expanded=nodes_expanded,
                    path_cost=int(g_score[current]),
                    duration_ms=duration_ms,
                    visited_nodes=visited_order,
                )

            current_g = g_score[current]

            for act, neighbor in self.grid.get_neighbors(current[0], current[1], self.direction_order):
                tentative_g = current_g + 1.0  # Unit step cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    parent_map[neighbor] = current
                    counter += 1
                    h_val = multi_goal_heuristic(neighbor, goals, self.metric)
                    f_score = tentative_g + h_val
                    heapq.heappush(frontier, (f_score, counter, neighbor))

        duration_ms = (time.perf_counter() - start_time) * 1000
        return SearchResult(
            algorithm="A*",
            success=False,
            nodes_expanded=nodes_expanded,
            duration_ms=duration_ms,
            visited_nodes=visited_order,
        )
