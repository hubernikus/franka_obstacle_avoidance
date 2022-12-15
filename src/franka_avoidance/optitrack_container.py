"""
Container which smoothenes position (and rotation) of incoming obstacles.
"""
import warnings

from typing import Protocol, Optional

# from enum import Enum, auto
import rclpy
from rclpy.node import Node

import numpy as np

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.containers import ObstacleContainer

from franka_avoidance.optitrack_interface import OptitrackInterface
from franka_avoidance.pybullet_handler import PybulletHandler
from franka_avoidance.rviz_handler import RvizHandler

from franka_avoidance.state_filters import PositionFilter, SimpleOrientationFilter


class VisualizationHandler(Protocol):
    """Visualization handler which allows updating / publishing the with corresponding
    visualization tool."""

    def update(self, obstacles: list[Obstacle], obstacle_ids: list[int]) -> None:
        ...


class OptitrackContainer(ObstacleContainer):
    def __init__(
        self,
        visualization_handler: Optional[VisualizationHandler] = None,
        use_optitrack: bool = True,
    ):
        super().__init__()
        obstacle_ids = []
        obstacle_offsets = []

        # Setup full optitrack callback
        self.use_optitrack = use_optitrack
        if use_optitrack:
            self.optitrack_reciever = OptitrackInterface()
        self.visualization_handler = visualization_handler

    def update_obstacles(self):
        """Update positions based on optitrack."""
        if self.visualization_handler is None:
            return

        obstacle_ids = np.arange(len(self))
        self.visualization_handler.update(self, obstacle_ids)

    def append(
        self,
        obstacle: Obstacle,
        obstacle_topic: "str" = None,
        # start_position: np.ndarray,
    ):
        super().append(obstacle)

    def update(self) -> None:
        if self.optitrack_interface is None:
            return

        # Set positions
        self.a = 0
        self.optitrack_reciever = None

        pass

    def shutdown(self):
        self.visualization_handler.remove_all_obstacles()
