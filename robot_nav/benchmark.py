"""
Benchmark Suite for Grid Navigation Search Algorithms.
Executes systematic evaluations across diverse map topologies, tracking
time complexity, space complexity, optimality, and node expansions.
Author: Abdul Rehman Rattu
"""

from typing import List, Dict, Any, Optional
import time
import pandas as pd
from robot_nav.grid import GridMap
from robot_nav.algorithms import (
    BreadthFirstSearch,
    DepthFirstSearch,
    GreedyBestFirstSearch,
    AStarSearch,
    DFSBFSHybridSearch,
    BidirectionalAStarSearch,
    SearchResult,
)


class BenchmarkSuite:
    """Automated benchmark runner for search algorithms."""

    ALGORITHM_MAP = {
        "BFS": BreadthFirstSearch,
        "DFS": DepthFirstSearch,
        "GBFS (Manhattan)": lambda g: GreedyBestFirstSearch(g, metric="manhattan"),
        "A* (Manhattan)": lambda g: AStarSearch(g, metric="manhattan"),
        "A* (Euclidean)": lambda g: AStarSearch(g, metric="euclidean"),
        "DFS-BFS Hybrid": DFSBFSHybridSearch,
        "Bidirectional A*": lambda g: BidirectionalAStarSearch(g, metric="manhattan"),
    }

    def __init__(self, grid_map: Optional[GridMap] = None):
        self.grid = grid_map

    def run_single_map(self, grid_map: Optional[GridMap] = None) -> pd.DataFrame:
        """Benchmark all 7 algorithm variations on a single grid map."""
        g = grid_map or self.grid
        if g is None:
            raise ValueError("No GridMap provided for benchmarking.")

        records = []
        for name, algo_factory in self.ALGORITHM_MAP.items():
            searcher = algo_factory(g)
            # Run multiple iterations for high precision timing
            trials = 3
            durations = []
            result: Optional[SearchResult] = None
            for _ in range(trials):
                t0 = time.perf_counter()
                res = searcher.search()
                t1 = time.perf_counter()
                durations.append((t1 - t0) * 1000)
                result = res

            avg_duration = sum(durations) / len(durations)

            records.append({
                "Algorithm": name,
                "Success": "Yes" if result.success else "No",
                "Nodes Expanded": result.nodes_expanded,
                "Path Cost (Steps)": result.path_cost if result.success else "N/A",
                "Runtime (ms)": round(avg_duration, 3),
                "Path Length": len(result.path) if result.success else 0,
            })

        df = pd.DataFrame(records)
        return df

    @staticmethod
    def generate_benchmark_maps() -> Dict[str, GridMap]:
        """Synthesize reproducible benchmarking maps with varying obstacle distributions."""
        maps = {}

        # 1. Sparse 20x20
        g_sparse = GridMap(
            width=20,
            height=20,
            start=(1, 1),
            goals=[(18, 18)],
            obstacles=[(x, 10) for x in range(3, 17) if x not in (9, 10)],
        )
        maps["Sparse Grid (20x20)"] = g_sparse

        # 2. Obstacle Barrier 25x15
        barrier_obs = [(12, y) for y in range(0, 11)] + [(18, y) for y in range(4, 15)]
        g_barrier = GridMap(
            width=25,
            height=15,
            start=(2, 2),
            goals=[(22, 12)],
            obstacles=barrier_obs,
        )
        maps["Staggered Barrier (25x15)"] = g_barrier

        # 3. Dense Maze 18x18
        maze_obs = []
        for x in range(2, 16, 2):
            for y in range(1, 15):
                if (x // 2 + y) % 5 != 0:
                    maze_obs.append((x, y))
        g_maze = GridMap(
            width=18,
            height=18,
            start=(0, 0),
            goals=[(17, 17)],
            obstacles=maze_obs,
        )
        maps["Maze Corridor (18x18)"] = g_maze

        return maps

    def run_multi_topology_benchmark(self) -> pd.DataFrame:
        """Run systematic benchmark across standard synthetic and loaded topologies."""
        all_maps = self.generate_benchmark_maps()
        if self.grid is not None:
            all_maps = {"Default Loaded Map": self.grid, **all_maps}

        records = []
        for map_name, g_map in all_maps.items():
            for algo_name, algo_factory in self.ALGORITHM_MAP.items():
                searcher = algo_factory(g_map)
                t0 = time.perf_counter()
                res = searcher.search()
                t1 = time.perf_counter()
                dur_ms = (t1 - t0) * 1000

                records.append({
                    "Topology": map_name,
                    "Algorithm": algo_name,
                    "Success": res.success,
                    "Nodes Expanded": res.nodes_expanded,
                    "Path Cost": res.path_cost if res.success else None,
                    "Runtime (ms)": round(dur_ms, 3),
                })

        return pd.DataFrame(records)

