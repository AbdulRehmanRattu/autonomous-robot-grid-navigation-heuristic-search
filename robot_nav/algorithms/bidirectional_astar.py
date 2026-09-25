"""
Bidirectional A* Search Algorithm for Grid Navigation.
Simultaneously initiates search from the start node and goal nodes,
dramatically reducing the search space radius from O(b^d) to O(2 * b^(d/2)).
Author: Abdul Rehman Rattu
"""

from typing import List, Tuple, Dict, Set, Optional
import heapq
import time
from robot_nav.algorithms.base import BaseSearcher, SearchResult
from robot_nav.heuristics import multi_goal_heuristic, manhattan_distance


class BidirectionalAStarSearch(BaseSearcher):
    """
    Bidirectional A* Search.
    Maintains dual search frontiers:
    1. Forward frontier advancing from Start to Goals.
    2. Backward frontier retreating from Goals to Start.
    Terminates when frontiers intersect, synthesizing an optimal path.
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
                algorithm="Bidirectional A*",
                success=True,
                path=[start],
                actions=[],
                nodes_expanded=1,
                path_cost=0,
                duration_ms=(time.perf_counter() - start_time) * 1000,
                visited_nodes=[start],
            )

        counter = 0

        # Forward frontier data structures
        g_fwd: Dict[Tuple[int, int], float] = {start: 0.0}
        f_start_fwd = multi_goal_heuristic(start, goals, self.metric)
        frontier_fwd = [(f_start_fwd, counter, start)]
        parent_fwd: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited_fwd: Set[Tuple[int, int]] = set()

        # Backward frontier data structures
        g_bwd: Dict[Tuple[int, int], float] = {}
        frontier_bwd = []
        parent_bwd: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited_bwd: Set[Tuple[int, int]] = set()

        for g in goals:
            g_bwd[g] = 0.0
            counter += 1
            f_start_bwd = manhattan_distance(g, start)
            heapq.heappush(frontier_bwd, (f_start_bwd, counter, g))

        visited_order: List[Tuple[int, int]] = []
        nodes_expanded = 0
        meeting_node: Optional[Tuple[int, int]] = None
        best_cost = float("inf")

        while frontier_fwd and frontier_bwd:
            # 1. Forward expansion step
            f_val_f, _, curr_fwd = heapq.heappop(frontier_fwd)
            if curr_fwd not in visited_fwd:
                visited_fwd.add(curr_fwd)
                nodes_expanded += 1
                visited_order.append(curr_fwd)

                # Check for intersection
                if curr_fwd in visited_bwd:
                    cost = g_fwd[curr_fwd] + g_bwd[curr_fwd]
                    if cost < best_cost:
                        best_cost = cost
                        meeting_node = curr_fwd
                        break

                for act, neighbor in self.grid.get_neighbors(curr_fwd[0], curr_fwd[1], self.direction_order):
                    tentative_g = g_fwd[curr_fwd] + 1.0
                    if neighbor not in g_fwd or tentative_g < g_fwd[neighbor]:
                        g_fwd[neighbor] = tentative_g
                        parent_fwd[neighbor] = curr_fwd
                        counter += 1
                        h_val = multi_goal_heuristic(neighbor, goals, self.metric)
                        heapq.heappush(frontier_fwd, (tentative_g + h_val, counter, neighbor))

            # 2. Backward expansion step
            f_val_b, _, curr_bwd = heapq.heappop(frontier_bwd)
            if curr_bwd not in visited_bwd:
                visited_bwd.add(curr_bwd)
                nodes_expanded += 1
                visited_order.append(curr_bwd)

                # Check for intersection
                if curr_bwd in visited_fwd:
                    cost = g_fwd[curr_bwd] + g_bwd[curr_bwd]
                    if cost < best_cost:
                        best_cost = cost
                        meeting_node = curr_bwd
                        break

                for act, neighbor in self.grid.get_neighbors(curr_bwd[0], curr_bwd[1], self.direction_order):
                    tentative_g = g_bwd[curr_bwd] + 1.0
                    if neighbor not in g_bwd or tentative_g < g_bwd[neighbor]:
                        g_bwd[neighbor] = tentative_g
                        parent_bwd[neighbor] = curr_bwd
                        counter += 1
                        h_val = manhattan_distance(neighbor, start)
                        heapq.heappush(frontier_bwd, (tentative_g + h_val, counter, neighbor))

        if meeting_node is not None:
            duration_ms = (time.perf_counter() - start_time) * 1000

            # Reconstruct forward path from start to meeting_node
            fwd_path, _ = self.reconstruct_path(parent_fwd, meeting_node, start)

            # Reconstruct backward path from meeting_node to goal
            bwd_segment = []
            curr = meeting_node
            while curr in parent_bwd:
                curr = parent_bwd[curr]
                bwd_segment.append(curr)

            full_path = fwd_path + bwd_segment

            # Compute action sequence
            actions = []
            for i in range(len(full_path) - 1):
                dx = full_path[i + 1][0] - full_path[i][0]
                dy = full_path[i + 1][1] - full_path[i][1]
                act = self.inverse_offsets.get((dx, dy), "unknown")
                actions.append(act)

            return SearchResult(
                algorithm="Bidirectional A*",
                success=True,
                path=full_path,
                actions=actions,
                nodes_expanded=nodes_expanded,
                path_cost=len(full_path) - 1,
                duration_ms=duration_ms,
                visited_nodes=visited_order,
            )

        duration_ms = (time.perf_counter() - start_time) * 1000
        return SearchResult(
            algorithm="Bidirectional A*",
            success=False,
            nodes_expanded=nodes_expanded,
            duration_ms=duration_ms,
            visited_nodes=visited_order,
        )
