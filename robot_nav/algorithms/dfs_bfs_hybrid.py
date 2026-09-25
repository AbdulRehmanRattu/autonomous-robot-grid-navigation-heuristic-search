"""
DFS-BFS Hybrid Search Algorithm for Grid Navigation.
Combines depth-first aggressive probing with breadth-first layer containment
to balance traversal depth and branching factor exploration.
Author: Abdul Rehman Rattu
"""

from typing import List, Tuple, Dict, Set
from collections import deque
import time
from robot_nav.algorithms.base import BaseSearcher, SearchResult


class DFSBFSHybridSearch(BaseSearcher):
    """
    Hybrid Search Strategy.
    Alternates between Depth-First Search (probing ahead along promising branches)
    and Breadth-First Search (sweeping intermediate frontier tiers) to mitigate
    the memory explosion of BFS while avoiding DFS infinite branch traps.
    """

    def __init__(self, grid_map, dfs_probe_depth: int = 4):
        super().__init__(grid_map)
        self.probe_depth = dfs_probe_depth

    def search(self) -> SearchResult:
        start_time = time.perf_counter()
        start = self.grid.start

        if self.grid.is_goal(start[0], start[1]):
            return SearchResult(
                algorithm="DFS-BFS Hybrid",
                success=True,
                path=[start],
                actions=[],
                nodes_expanded=1,
                path_cost=0,
                duration_ms=(time.perf_counter() - start_time) * 1000,
                visited_nodes=[start],
            )

        # Main breadth queue of anchor points
        bfs_queue = deque([start])
        visited: Set[Tuple[int, int]] = {start}
        parent_map: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited_order: List[Tuple[int, int]] = []
        nodes_expanded = 0

        while bfs_queue:
            anchor = bfs_queue.popleft()
            nodes_expanded += 1
            visited_order.append(anchor)

            if self.grid.is_goal(anchor[0], anchor[1]):
                duration_ms = (time.perf_counter() - start_time) * 1000
                path, actions = self.reconstruct_path(parent_map, anchor, start)
                return SearchResult(
                    algorithm="DFS-BFS Hybrid",
                    success=True,
                    path=path,
                    actions=actions,
                    nodes_expanded=nodes_expanded,
                    path_cost=len(path) - 1,
                    duration_ms=duration_ms,
                    visited_nodes=visited_order,
                )

            # Local DFS probe from this anchor up to probe_depth
            dfs_stack: List[Tuple[Tuple[int, int], int]] = [(anchor, 0)]
            leaf_nodes: List[Tuple[int, int]] = []

            while dfs_stack:
                current, depth = dfs_stack.pop()

                if current != anchor and current not in visited_order:
                    nodes_expanded += 1
                    visited_order.append(current)

                if self.grid.is_goal(current[0], current[1]):
                    duration_ms = (time.perf_counter() - start_time) * 1000
                    path, actions = self.reconstruct_path(parent_map, current, start)
                    return SearchResult(
                        algorithm="DFS-BFS Hybrid",
                        success=True,
                        path=path,
                        actions=actions,
                        nodes_expanded=nodes_expanded,
                        path_cost=len(path) - 1,
                        duration_ms=duration_ms,
                        visited_nodes=visited_order,
                    )

                if depth < self.probe_depth:
                    # Expand using reversed direction order for DFS stack order consistency
                    neighbors = self.grid.get_neighbors(current[0], current[1], self.direction_order)
                    expanded_any = False
                    for act, neighbor in reversed(neighbors):
                        if neighbor not in visited:
                            visited.add(neighbor)
                            parent_map[neighbor] = current
                            dfs_stack.append((neighbor, depth + 1))
                            expanded_any = True
                    if not expanded_any and depth > 0:
                        leaf_nodes.append(current)
                else:
                    leaf_nodes.append(current)

            # Add deep frontier leaves to the BFS queue for systemic regional coverage
            for leaf in leaf_nodes:
                neighbors = self.grid.get_neighbors(leaf[0], leaf[1], self.direction_order)
                for act, neighbor in neighbors:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        parent_map[neighbor] = leaf
                        bfs_queue.append(neighbor)

        duration_ms = (time.perf_counter() - start_time) * 1000
        return SearchResult(
            algorithm="DFS-BFS Hybrid",
            success=False,
            nodes_expanded=nodes_expanded,
            duration_ms=duration_ms,
            visited_nodes=visited_order,
        )
