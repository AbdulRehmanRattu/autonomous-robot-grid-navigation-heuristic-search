"""
Grid Map representation and coordinate file parser for Autonomous Robot Navigation.
Author: Abdul Rehman Rattu
"""

from typing import List, Tuple, Optional, Set
import os
import ast
import numpy as np


class GridMap:
    """
    2D Discrete Occupancy Grid for Robot Navigation.
    Cell values:
        0: Free traversable space
        1: Obstacle / Wall
        2: Initial robot position (Start)
        3: Goal destination
    """

    def __init__(
        self,
        width: int,
        height: int,
        start: Tuple[int, int] = (0, 0),
        goals: Optional[List[Tuple[int, int]]] = None,
        obstacles: Optional[List[Tuple[int, int]]] = None,
    ):
        self.width = int(width)
        self.height = int(height)
        # grid[x][y] representation
        self.grid = np.zeros((self.width, self.height), dtype=np.int32)
        self.start: Tuple[int, int] = (int(start[0]), int(start[1]))
        self.grid[self.start[0], self.start[1]] = 2
        self.goals: List[Tuple[int, int]] = []
        if goals:
            for g in goals:
                self.add_goal(g[0], g[1])
        if obstacles:
            for ox, oy in obstacles:
                if 0 <= ox < self.width and 0 <= oy < self.height:
                    self.grid[ox, oy] = 1

    @property
    def obstacles(self) -> Set[Tuple[int, int]]:
        """Return set of all obstacle coordinates."""
        xs, ys = np.where(self.grid == 1)
        return set(zip(xs, ys))

    def set_start(self, x: int, y: int) -> None:
        """Assign start location."""
        self.start = (int(x), int(y))
        self.grid[int(x), int(y)] = 2

    def add_goal(self, x: int, y: int) -> None:
        """Register goal destination."""
        goal = (int(x), int(y))
        if goal not in self.goals:
            self.goals.append(goal)
        self.grid[int(x), int(y)] = 3


    def set_wall_rect(self, x: int, y: int, w: int, h: int) -> None:
        """Add rectangular obstacle block [x, y, w, h]."""
        for r_x in range(x, x + w):
            for r_y in range(y, y + h):
                if 0 <= r_x < self.width and 0 <= r_y < self.height:
                    self.grid[r_x, r_y] = 1

    def toggle_wall(self, x: int, y: int) -> None:
        """Toggle wall status for interactive UI clicks."""
        if 0 <= x < self.width and 0 <= y < self.height:
            if (x, y) != self.start and (x, y) not in self.goals:
                self.grid[x, y] = 1 if self.grid[x, y] == 0 else 0

    def is_valid(self, x: int, y: int) -> bool:
        """Check boundary bounds."""
        return 0 <= x < self.width and 0 <= y < self.height

    def is_walkable(self, x: int, y: int) -> bool:
        """Check if cell is within bounds and not an obstacle."""
        return self.is_valid(x, y) and self.grid[x, y] != 1

    def is_goal(self, x: int, y: int) -> bool:
        """Check if position is among target goals."""
        return (x, y) in self.goals or (self.is_valid(x, y) and self.grid[x, y] == 3)

    def get_neighbors(self, x: int, y: int, order: Tuple[str, ...] = ("up", "left", "down", "right")) -> List[Tuple[str, Tuple[int, int]]]:
        """
        Return legal 4-directional transitions in standardized priority order:
        Up (0, -1), Left (-1, 0), Down (0, 1), Right (1, 0).
        """
        offsets = {
            "up": (0, -1),
            "left": (-1, 0),
            "down": (0, 1),
            "right": (1, 0),
        }
        neighbors = []
        for direction in order:
            dx, dy = offsets[direction]
            nx, ny = x + dx, y + dy
            if self.is_walkable(nx, ny):
                neighbors.append((direction, (nx, ny)))
        return neighbors

    @classmethod
    def from_file(cls, filepath: str) -> "GridMap":
        """
        Parse benchmark map file format:
            Line 1: [height, width]
            Line 2: (start_x, start_y)
            Line 3: (goal1_x, goal1_y) | (goal2_x, goal2_y)
            Lines 4+: (wall_x, wall_y, wall_w, wall_h)
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Map file not found: {filepath}")

        with open(filepath, "r") as f:
            lines = [l.strip() for l in f.readlines() if l.strip()]

        # Parse grid size [rows, cols] -> [height, width]
        dims = lines[0].strip("[]").split(",")
        height, width = int(dims[0].strip()), int(dims[1].strip())
        grid_map = cls(width=width, height=height)

        # Parse Start coordinate
        start_parts = lines[1].strip("()").split(",")
        sx, sy = int(start_parts[0].strip()), int(start_parts[1].strip())
        grid_map.set_start(sx, sy)

        # Parse Goals (supports multiple pipe-separated goals)
        goal_tokens = lines[2].replace("|", " ").split()
        for token in goal_tokens:
            gx, gy = ast.literal_eval(token)
            grid_map.add_goal(gx, gy)

        # Parse Obstacle Rectangles
        for line in lines[3:]:
            parts = line.strip("()").split(",")
            if len(parts) == 4:
                wx, wy, ww, wh = [int(p.strip()) for p in parts]
                grid_map.set_wall_rect(wx, wy, ww, wh)

        return grid_map

    @classmethod
    def create_warehouse_benchmark(cls) -> "GridMap":
        """Generate complex automated warehouse grid with shelving corridors."""
        g = cls(width=25, height=18)
        g.set_start(1, 1)
        g.add_goal(23, 16)
        g.add_goal(23, 2)

        # Shelving racks
        for row in [3, 7, 11, 15]:
            for col_start in [3, 9, 15]:
                g.set_wall_rect(col_start, row, 4, 1)

        # Perimeter pillars
        g.set_wall_rect(7, 5, 2, 2)
        g.set_wall_rect(17, 9, 2, 2)
        return g
