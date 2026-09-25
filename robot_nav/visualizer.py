"""
Publication-Grade Visualizer for Search Algorithms.
Renders 300 DPI architecture schematics and empirical benchmark analytics
following high-standard design guidelines (white background, clean typography).
Author: Abdul Rehman Rattu
"""

import os
from typing import Dict, List, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd


def generate_architecture_diagram(output_path: str):
    """
    Renders a crisp 300 DPI system architecture diagram.
    Uses clean white background, professional palettes, and structured layout.
    """
    fig, ax = plt.subplots(figsize=(14, 9), dpi=300)
    fig.patch.set_facecolor("#FFFFFF")
    ax.set_facecolor("#FFFFFF")
    ax.axis("off")
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 9)

    # Title Banner
    ax.text(
        7.0, 8.55,
        "AUTONOMOUS ROBOT SEARCH & NAVIGATION SYSTEM ARCHITECTURE",
        ha="center", va="center", fontsize=15, fontweight="bold", color="#0F172A",
        fontfamily="sans-serif"
    )
    ax.text(
        7.0, 8.25,
        "Modular Multi-Algorithm Graph Traversal, Heuristic Evaluation, and Empirical Telemetry Pipeline",
        ha="center", va="center", fontsize=10, color="#64748B", fontfamily="sans-serif"
    )

    # Define Layers
    layers = [
        {
            "y": 6.8, "h": 1.15, "title": "LAYER 1: ENVIRONMENT & SPATIAL TOPOLOGY",
            "bg": "#F8FAFC", "border": "#94A3B8", "badge": "#475569",
            "components": [
                ("Grid Matrix & Parser", "Dimension (W x H)\nASCII & File Descriptors\nDynamic Obstacle Maps"),
                ("Spatial Coordinate Model", "Cartesian (X, Y)\nOrigin at Top-Left\nDirect Grid Indexing"),
                ("Kinematic Transitions", "4-Way Discrete Movements\nUp: (0,-1) | Left: (-1,0)\nDown: (0,1) | Right: (1,0)"),
                ("Multi-Goal Anchors", "Primary/Secondary Targets\nDynamic Goal Sets\nEuclidean Goal Receptors"),
            ]
        },
        {
            "y": 5.25, "h": 1.15, "title": "LAYER 2: HEURISTIC EVALUATION ENGINE",
            "bg": "#EFF6FF", "border": "#60A5FA", "badge": "#2563EB",
            "components": [
                ("Manhattan Metric (L1)", "h(n) = |x - gx| + |y - gy|\nAdmissible for 4-way Grid\nGuarantees A* Optimality"),
                ("Euclidean Metric (L2)", "h(n) = sqrt(dx^2 + dy^2)\nStraight-line relaxation\nConsistent & monotone"),
                ("Chebyshev Metric (Linf)", "h(n) = max(|dx|, |dy|)\nDiagonal navigation bound\nConservative lower bound"),
                ("Multi-Goal Minimizer", "h*(n) = min_{g} h(n, g)\nSelects closest target\nOptimal multi-agent routing"),
            ]
        },
        {
            "y": 3.7, "h": 1.15, "title": "LAYER 3: DISCRETE GRAPH SEARCH ALGORITHMS",
            "bg": "#F0FDF4", "border": "#4ADE80", "badge": "#16A34A",
            "components": [
                ("Uninformed Traversal", "BFS (FIFO Queue - Optimal)\nDFS (LIFO Stack - Deep Probe)\nDeterministic Tie-Breaking"),
                ("Greedy Best-First (GBFS)", "Priority Queue: f(n) = h(n)\nRapid Goal Convergence\nSub-optimal Path Tradeoff"),
                ("A* Optimal Search", "f(n) = g(n) + h(n)\nOptimal & Complete\nBranch Pruning via g-scores"),
                ("Hybrid & Bidirectional", "DFS-BFS Stratified Hybrid\nBidirectional A* Dual Frontiers\nRadical Search Space Halving"),
            ]
        },
        {
            "y": 2.15, "h": 1.15, "title": "LAYER 4: SOLUTION SYNTHESIS & TELEMETRY",
            "bg": "#FAF5FF", "border": "#C084FC", "badge": "#9333EA",
            "components": [
                ("Path Backtracking", "Parent-pointer inversion\nStart-to-Goal node path\nCycle avoidance checks"),
                ("Action Synthesizer", "Generates execution vector:\n[up, left, down, right]\nKinematic validation"),
                ("Complexity Telemetry", "Expanded Nodes Count\nStep Cost & Trajectory\nNanosecond Wall-time (ms)"),
                ("Optimality Verifier", "Theoretical vs Empirical\nBranching Factor b*\nRedundant Step Inspector"),
            ]
        },
        {
            "y": 0.6, "h": 1.15, "title": "LAYER 5: EXECUTION SURFACES & INTERACTION",
            "bg": "#FFFBEB", "border": "#FBBF24", "badge": "#D97706",
            "components": [
                ("Production CLI", "Modular run_pipeline.py\nMap file ingestion\nBatch execution modes"),
                ("Benchmark Suite", "Multi-topology stress tests\nAutomated comparison tables\nStatistical aggregations"),
                ("Pygame Desktop GUI", "Real-time step visualization\nInteractive wall editing\nDynamic start/goal dragging"),
                ("Publication Visuals", "300 DPI Export Engine\nPareto Frontier Curves\nDetailed Technical Reports"),
            ]
        },
    ]

    for layer in layers:
        y = layer["y"]
        h = layer["h"]
        # Container Box
        box = patches.FancyBboxPatch(
            (0.5, y), 13.0, h,
            boxstyle="round,pad=0.08,rounding_size=0.15",
            facecolor=layer["bg"],
            edgecolor=layer["border"],
            linewidth=1.4,
        )
        ax.add_patch(box)

        # Layer Tag
        tag = patches.FancyBboxPatch(
            (0.7, y + h - 0.22), 3.4, 0.26,
            boxstyle="round,pad=0.04,rounding_size=0.08",
            facecolor=layer["badge"],
            edgecolor="none",
        )
        ax.add_patch(tag)
        ax.text(
            0.75, y + h - 0.09, layer["title"],
            fontsize=7.8, fontweight="bold", color="#FFFFFF", fontfamily="sans-serif", va="center"
        )

        # 4 Sub-components per layer
        n_comps = len(layer["components"])
        comp_w = 2.85
        spacing = (13.0 - (comp_w * n_comps)) / (n_comps + 1)

        for i, (head, body) in enumerate(layer["components"]):
            cx = 0.5 + spacing + i * (comp_w + spacing)
            cy = y + 0.12
            c_box = patches.FancyBboxPatch(
                (cx, cy), comp_w, h - 0.42,
                boxstyle="round,pad=0.04,rounding_size=0.08",
                facecolor="#FFFFFF",
                edgecolor=layer["border"],
                linewidth=0.8,
            )
            ax.add_patch(c_box)

            # Sub-component header
            ax.text(
                cx + comp_w / 2, cy + (h - 0.42) - 0.18, head,
                fontsize=8.2, fontweight="bold", color="#1E293B", ha="center", va="center"
            )
            # Sub-component body
            ax.text(
                cx + comp_w / 2, cy + ((h - 0.42) / 2) - 0.1, body,
                fontsize=6.8, color="#475569", ha="center", va="center", multialignment="center"
            )

    # Clean connecting flow arrows on left margin
    for i in range(len(layers) - 1):
        y_from = layers[i]["y"]
        y_to = layers[i + 1]["y"] + layers[i + 1]["h"]
        arrow = patches.FancyArrowPatch(
            (0.35, y_from), (0.35, y_to),
            arrowstyle="->,head_width=3,head_length=4",
            color="#64748B",
            linewidth=1.2,
        )
        ax.add_patch(arrow)

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=300, facecolor="#FFFFFF", edgecolor="none")
    plt.close()
    print(f"[Visualizer] Saved architecture diagram: {output_path}")


def generate_benchmark_diagram(benchmark_df: pd.DataFrame, output_path: str):
    """
    Renders a 4-panel empirical benchmark visualization at 300 DPI.
    Panels:
    1. Nodes Expanded (Search Space Exploration)
    2. Path Cost (Optimality Verification)
    3. Computational Latency (ms)
    4. Pareto Efficiency (Nodes Expanded vs Path Cost)
    """
    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
    fig, axes = plt.subplots(2, 2, figsize=(15, 11), dpi=300)
    fig.patch.set_facecolor("#FFFFFF")

    for row in axes:
        for ax in row:
            ax.set_facecolor("#FFFFFF")

    # Filter to successful runs
    success_df = benchmark_df[benchmark_df["Success"] == "Yes"].copy()
    if success_df.empty:
        success_df = benchmark_df.copy()

    algos = success_df["Algorithm"].tolist()
    nodes = success_df["Nodes Expanded"].tolist()
    costs = [c if isinstance(c, (int, float)) else 0 for c in success_df["Path Cost (Steps)"].tolist()]
    runtimes = success_df["Runtime (ms)"].tolist()

    palette = ["#2563EB", "#DC2626", "#059669", "#7C3AED", "#D97706", "#0891B2", "#4F46E5"]
    colors = [palette[i % len(palette)] for i in range(len(algos))]

    # Panel 1: Nodes Expanded
    ax1 = axes[0, 0]
    bars1 = ax1.bar(algos, nodes, color=colors, alpha=0.88, edgecolor="#1E293B", linewidth=0.8, width=0.55)
    ax1.set_title("Search Efficiency: Nodes Expanded (Lower is Better)", fontsize=11, fontweight="bold", pad=12)
    ax1.set_ylabel("Nodes Expanded", fontsize=9, fontweight="bold")
    ax1.tick_params(axis="x", rotation=25, labelsize=8)
    for bar in bars1:
        yval = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width() / 2, yval + (max(nodes) * 0.02), f"{int(yval)}",
                 ha="center", va="bottom", fontsize=8, fontweight="bold")

    # Panel 2: Path Cost (Optimality)
    ax2 = axes[0, 1]
    bars2 = ax2.bar(algos, costs, color=colors, alpha=0.88, edgecolor="#1E293B", linewidth=0.8, width=0.55)
    ax2.set_title("Path Cost Optimality (Unit Steps)", fontsize=11, fontweight="bold", pad=12)
    ax2.set_ylabel("Steps to Goal", fontsize=9, fontweight="bold")
    ax2.tick_params(axis="x", rotation=25, labelsize=8)
    min_cost = min([c for c in costs if c > 0]) if any(c > 0 for c in costs) else 0
    ax2.axhline(min_cost, color="#10B981", linestyle="--", linewidth=1.5, label=f"Optimal Bound ({min_cost} steps)")
    ax2.legend(loc="upper right", fontsize=8)
    for bar in bars2:
        yval = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width() / 2, yval + 0.3, f"{int(yval)}",
                 ha="center", va="bottom", fontsize=8, fontweight="bold")

    # Panel 3: Execution Runtime (ms)
    ax3 = axes[1, 0]
    bars3 = ax3.bar(algos, runtimes, color=colors, alpha=0.88, edgecolor="#1E293B", linewidth=0.8, width=0.55)
    ax3.set_title("Computational Latency: Execution Time (ms)", fontsize=11, fontweight="bold", pad=12)
    ax3.set_ylabel("Runtime (Milliseconds)", fontsize=9, fontweight="bold")
    ax3.tick_params(axis="x", rotation=25, labelsize=8)
    for bar in bars3:
        yval = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width() / 2, yval + (max(runtimes) * 0.02), f"{yval:.2f}ms",
                 ha="center", va="bottom", fontsize=8, fontweight="bold")

    # Panel 4: Pareto Frontier: Nodes Expanded vs Path Cost
    ax4 = axes[1, 1]
    offsets_map = {
        "BFS": (10, -5),
        "DFS": (10, -3),
        "GBFS (Manhattan)": (10, 8),
        "A* (Manhattan)": (-20, 16),
        "A* (Euclidean)": (10, 12),
        "DFS-BFS Hybrid": (10, 6),
        "Bidirectional A*": (-75, -20),
    }

    for i, algo in enumerate(algos):
        ax4.scatter(nodes[i], costs[i], color=colors[i], s=140, edgecolor="#1E293B", linewidth=1.2, zorder=5)
        offset = offsets_map.get(algo, (8, 6))
        ax4.annotate(
            algo,
            (nodes[i], costs[i]),
            xytext=offset,
            textcoords="offset points",
            fontsize=8.5,
            fontweight="bold",
            color="#1E293B",
            arrowprops=dict(arrowstyle="->", color="#94A3B8", lw=0.8) if offset[1] < 0 or offset[0] < 0 else None,
        )
    ax4.set_title("Pareto Efficiency: Search Effort vs. Path Optimality", fontsize=11, fontweight="bold", pad=12)
    ax4.set_xlabel("Nodes Expanded (Search Effort)", fontsize=9, fontweight="bold")
    ax4.set_ylabel("Path Cost (Solution Quality)", fontsize=9, fontweight="bold")
    ax4.set_ylim(8, 22)
    ax4.set_xlim(10, 36)


    plt.suptitle("AUTONOMOUS ROBOT SEARCH ALGORITHM BENCHMARK SUITE", fontsize=14, fontweight="bold", y=0.99)
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=300, facecolor="#FFFFFF", edgecolor="none")
    plt.close()
    print(f"[Visualizer] Saved benchmark diagram: {output_path}")
