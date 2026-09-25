#!/usr/bin/env python3
"""
Interactive Desktop Simulation GUI for Grid Navigation.
Real-time step-by-step frontier expansion, obstacle painting, and path tracing.

Author: Abdul Rehman Rattu
License: MIT
"""

import sys
import os
from pathlib import Path
import time

# Add root directory
sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_nav.grid import GridMap
from robot_nav.algorithms import (
    BreadthFirstSearch,
    DepthFirstSearch,
    GreedyBestFirstSearch,
    AStarSearch,
    DFSBFSHybridSearch,
    BidirectionalAStarSearch,
)


def run_gui():
    try:
        import pygame
    except ImportError:
        print("[Warning] 'pygame' is not installed. To launch the interactive desktop GUI, run:")
        print("  pip install pygame")
        sys.exit(0)

    # Initialize Pygame
    pygame.init()
    pygame.font.init()

    # Load initial default map or fallback to clean 20x15 grid
    default_map_path = Path("data/maps/benchmark_default.txt")
    if default_map_path.exists():
        grid = GridMap.from_file(str(default_map_path))
    else:
        grid = GridMap(20, 15, (1, 1), [(18, 13)])

    cell_size = 40
    hud_height = 90
    screen_width = max(grid.width * cell_size, 640)
    screen_height = grid.height * cell_size + hud_height

    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Autonomous Robot Grid Navigation Suite - Abdul Rehman Rattu")
    clock = pygame.time.Clock()

    # Color Palette
    BG_COLOR = (248, 250, 252)       # Light Slate
    HUD_BG = (15, 23, 42)            # Dark Slate
    GRID_LINE = (226, 232, 240)      # Border
    WALL_COLOR = (30, 41, 59)        # Charcoal Wall
    START_COLOR = (16, 185, 129)     # Emerald Green
    GOAL_COLOR = (239, 68, 68)       # Crimson Red
    VISITED_COLOR = (186, 230, 253)  # Soft Sky Blue
    PATH_COLOR = (245, 158, 11)      # Vibrant Amber
    TEXT_COLOR = (255, 255, 255)
    TEXT_MUTED = (148, 163, 184)

    font_main = pygame.font.SysFont("Helvetica, Arial", 18, bold=True)
    font_sub = pygame.font.SysFont("Helvetica, Arial", 14)

    current_algo = "A*"
    search_result = None
    step_index = 0
    animating = False

    def solve(algo_name: str):
        nonlocal search_result, current_algo, step_index, animating
        current_algo = algo_name
        if algo_name == "BFS":
            runner = BreadthFirstSearch(grid)
        elif algo_name == "DFS":
            runner = DepthFirstSearch(grid)
        elif algo_name == "GBFS":
            runner = GreedyBestFirstSearch(grid, metric="manhattan")
        elif algo_name == "A*":
            runner = AStarSearch(grid, metric="manhattan")
        elif algo_name == "DFS-BFS Hybrid":
            runner = DFSBFSHybridSearch(grid)
        elif algo_name == "Bidirectional A*":
            runner = BidirectionalAStarSearch(grid, metric="manhattan")
        else:
            return

        search_result = runner.search()
        step_index = 0
        animating = True

    # Pre-solve with A*
    solve("A*")

    running = True
    while running:
        clock.tick(60)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_1:
                    solve("BFS")
                elif event.key == pygame.K_2:
                    solve("DFS")
                elif event.key == pygame.K_3:
                    solve("GBFS")
                elif event.key == pygame.K_4:
                    solve("A*")
                elif event.key == pygame.K_5:
                    solve("DFS-BFS Hybrid")
                elif event.key == pygame.K_6:
                    solve("Bidirectional A*")
                elif event.key == pygame.K_SPACE:
                    animating = not animating
                elif event.key == pygame.K_r:
                    search_result = None
                    step_index = 0
                    animating = False
                elif event.key == pygame.K_c:
                    grid.obstacles = set()
                    search_result = None
                    step_index = 0
                    animating = False

            # Obstacle drawing with mouse
            elif pygame.mouse.get_pressed()[0]:  # Left click to add wall
                mx, my = pygame.mouse.get_pos()
                if my >= hud_height:
                    gx = mx // cell_size
                    gy = (my - hud_height) // cell_size
                    if 0 <= gx < grid.width and 0 <= gy < grid.height:
                        if (gx, gy) != grid.start and (gx, gy) not in grid.goals:
                            grid.obstacles.add((gx, gy))
                            solve(current_algo)

            elif pygame.mouse.get_pressed()[2]:  # Right click to remove wall
                mx, my = pygame.mouse.get_pos()
                if my >= hud_height:
                    gx = mx // cell_size
                    gy = (my - hud_height) // cell_size
                    if (gx, gy) in grid.obstacles:
                        grid.obstacles.remove((gx, gy))
                        solve(current_algo)

        # Advance animation
        if animating and search_result and step_index < len(search_result.visited_nodes):
            step_index = min(step_index + 2, len(search_result.visited_nodes))

        # Render Frame
        screen.fill(BG_COLOR)

        # Draw HUD
        pygame.draw.rect(screen, HUD_BG, (0, 0, screen_width, hud_height))

        # HUD Titles
        title_surf = font_main.render(f"Algorithm: {current_algo}", True, TEXT_COLOR)
        screen.blit(title_surf, (16, 12))

        controls_surf = font_sub.render(
            "Keys: [1] BFS | [2] DFS | [3] GBFS | [4] A* | [5] Hybrid | [6] Bi-A* | [R] Reset | [C] Clear | [Space] Pause",
            True, TEXT_MUTED
        )
        screen.blit(controls_surf, (16, 38))

        if search_result:
            status_text = "SOLVED" if search_result.success else "NO PATH"
            stats_surf = font_sub.render(
                f"Status: {status_text} | Nodes Expanded: {search_result.nodes_expanded} | "
                f"Path Cost: {search_result.path_cost} steps | Latency: {search_result.duration_ms:.2f} ms",
                True, (52, 211, 153) if search_result.success else (248, 113, 113)
            )
            screen.blit(stats_surf, (16, 62))

        # Draw Grid Cells
        active_visited = set(search_result.visited_nodes[:step_index]) if search_result else set()
        path_set = set(search_result.path) if (search_result and step_index >= len(search_result.visited_nodes)) else set()

        for y in range(grid.height):
            for x in range(grid.width):
                rect = (x * cell_size, hud_height + y * cell_size, cell_size, cell_size)
                pos = (x, y)

                if pos == grid.start:
                    pygame.draw.rect(screen, START_COLOR, rect)
                elif pos in grid.goals:
                    pygame.draw.rect(screen, GOAL_COLOR, rect)
                elif pos in grid.obstacles:
                    pygame.draw.rect(screen, WALL_COLOR, rect)
                elif pos in path_set:
                    pygame.draw.rect(screen, PATH_COLOR, rect)
                elif pos in active_visited:
                    pygame.draw.rect(screen, VISITED_COLOR, rect)
                else:
                    pygame.draw.rect(screen, BG_COLOR, rect)

                pygame.draw.rect(screen, GRID_LINE, rect, 1)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    run_gui()
