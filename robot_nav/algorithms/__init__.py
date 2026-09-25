"""
Search Algorithms Package.
Exports unified interface for all 6 discrete navigation algorithms:
BFS, DFS, GBFS, A*, DFS-BFS Hybrid, Bidirectional A*.
Author: Abdul Rehman Rattu
"""

from robot_nav.algorithms.base import BaseSearcher, SearchResult
from robot_nav.algorithms.bfs import BreadthFirstSearch
from robot_nav.algorithms.dfs import DepthFirstSearch
from robot_nav.algorithms.gbfs import GreedyBestFirstSearch
from robot_nav.algorithms.astar import AStarSearch
from robot_nav.algorithms.dfs_bfs_hybrid import DFSBFSHybridSearch
from robot_nav.algorithms.bidirectional_astar import BidirectionalAStarSearch

__all__ = [
    "BaseSearcher",
    "SearchResult",
    "BreadthFirstSearch",
    "DepthFirstSearch",
    "GreedyBestFirstSearch",
    "AStarSearch",
    "DFSBFSHybridSearch",
    "BidirectionalAStarSearch",
]
