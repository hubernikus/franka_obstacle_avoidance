"""
Container which smoothenes position (and rotation) of incoming obstacles.
"""
import warnings

# from enum import Enum, auto

import numpy as np

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.containers import ObstacleContainer

from franka_avoidance.optitrack_interface import OptitrackInterface
from franka_avoidance.pybullet_handler import PybulletHandler


class RvizHandler:
    def __init__(self, obstacles: ObstacleContainer):
        pass

    def update(self, obstacles: ObstacleContainer):
        pass


# class VisualizationHandler(Protocol):
#     def __init__(self, obstacles: ObstacleContainer):
#         ...

#     def update(self, obstacles: ObstacleContainer):
#         ...

# class VisualizationMode(Enum):
#     RVIZ = auto()
#     PYBULLET = auto()
#     NONE = auto()


class OptitrackContainer(ObstacleContainer):
    def __init__(self, visualization_handler=None, use_optitrack: bool = True):
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
        self.visualization_handler.update(self)

    def append(
        self,
        obstacle: Obstacle,
        obstacle_id: int,
        # start_position: np.ndarray,
    ):
        super().append(obstacle)

    def callback(self):
        pass


if (__name__) == "__main__":
    obstacles = OptitrackContainer(use_optitrack=False)
    obstacles.append(
        Ellipse(
            center_position=np.array([1, 0, 1]),
            axes_length=np.array([1, 1, 1])
            # orientation
            # name="",
        ),
        obstacle_id=0,
    )

    obstacles.visualization_handler = PybulletHandler(obstacles)

    import time

    for ii in range(100):
        obstacles.update_obstacles()
        time.sleep(0.2)
