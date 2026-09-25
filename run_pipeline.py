#!/usr/bin/env python3
"""
Autonomous Robot Grid Navigation & Heuristic Search Suite.
Production CLI entry point for path planning, empirical benchmarking,
and high-resolution visualization generation.

Author: Abdul Rehman Rattu
License: MIT
"""

import argparse
import sys
import os
from pathlib import Path
import pandas as pd

# Add repository root to path
sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_nav.grid import GridMap
from robot_nav.heuristics import multi_goal_heuristic
from robot_nav.algorithms import (
    BreadthFirstSearch,
    DepthFirstSearch,
    GreedyBestFirstSearch,
    AStarSearch,
    DFSBFSHybridSearch,
    BidirectionalAStarSearch,
)
from robot_nav.benchmark import BenchmarkSuite
from robot_nav.visualizer import generate_architecture_diagram, generate_benchmark_diagram


def get_searcher(name: str, grid_map: GridMap, metric: str = "manhattan"):
    name = name.lower()
    if name == "bfs":
        return BreadthFirstSearch(grid_map)
    elif name == "dfs":
        return DepthFirstSearch(grid_map)
    elif name == "gbfs":
        return GreedyBestFirstSearch(grid_map, metric=metric)
    elif name == "astar" or name == "a*":
        return AStarSearch(grid_map, metric=metric)
    elif name in ("hybrid", "dfs-bfs", "dfs_bfs"):
        return DFSBFSHybridSearch(grid_map)
    elif name in ("bidirectional", "biastar", "bi-astar"):
        return BidirectionalAStarSearch(grid_map, metric=metric)
    else:
        raise ValueError(f"Unknown algorithm: {name}")


def main():
    parser = argparse.ArgumentParser(
        description="Autonomous Robot Grid Navigation & Heuristic Search Suite - Abdul Rehman Rattu",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--map",
        type=str,
        default="data/maps/benchmark_default.txt",
        help="Path to grid map definition file.",
    )
    parser.add_argument(
        "--algorithm",
        type=str,
        default="all",
        choices=["all", "bfs", "dfs", "gbfs", "astar", "hybrid", "bidirectional"],
        help="Search algorithm to execute.",
    )
    parser.add_argument(
        "--metric",
        type=str,
        default="manhattan",
        choices=["manhattan", "euclidean", "chebyshev"],
        help="Heuristic distance metric for informed search.",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="benchmark",
        choices=["solve", "benchmark", "visuals"],
        help="Execution mode: single solve, full benchmark, or generate visuals.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="assets/docs",
        help="Directory to save figures and artifacts.",
    )

    args = parser.parse_args()

    print("=" * 75)
    print("  AUTONOMOUS ROBOT GRID NAVIGATION & HEURISTIC SEARCH SUITE")
    print("  Engineering Lead: Abdul Rehman Rattu")
    print("=" * 75)

    map_path = Path(args.map)
    if not map_path.exists():
        print(f"[Error] Map file not found: {map_path}")
        sys.exit(1)

    grid = GridMap.from_file(str(map_path))
    print(f"[Environment] Loaded Grid: {grid.width}x{grid.height} | Start: {grid.start} | Goals: {grid.goals} | Obstacles: {len(grid.obstacles)} cells")

    if args.mode == "solve":
        if args.algorithm == "all":
            algos = ["bfs", "dfs", "gbfs", "astar", "hybrid", "bidirectional"]
        else:
            algos = [args.algorithm]

        for algo in algos:
            searcher = get_searcher(algo, grid, metric=args.metric)
            res = searcher.search()
            print("-" * 75)
            print(res.summary())
            if res.success:
                print(f"Path Coordinates ({len(res.path)} nodes): {res.path}")

    elif args.mode == "benchmark":
        suite = BenchmarkSuite(grid)
        df = suite.run_single_map()
        print("\n" + "=" * 75)
        print("  EMPIRICAL SEARCH BENCHMARK RESULTS")
        print("=" * 75)
        print(df.to_markdown(index=False))

    elif args.mode == "visuals":
        out_dir = Path(args.output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        arch_path = str(out_dir / "search_algorithms_architecture.png")
        bench_path = str(out_dir / "empirical_search_benchmark.png")

        print(f"[Visualizer] Generating 300 DPI architecture diagram at {arch_path}...")
        generate_architecture_diagram(arch_path)

        print("[Visualizer] Running benchmark to generate empirical telemetry...")
        suite = BenchmarkSuite(grid)
        df = suite.run_single_map()

        print(f"[Visualizer] Generating 300 DPI benchmark analytics at {bench_path}...")
        generate_benchmark_diagram(df, bench_path)

        print("[Visualizer] Visuals successfully rendered and saved.")

    print("=" * 75)
    print("  Execution completed successfully.")
    print("=" * 75)


if __name__ == "__main__":
    main()
