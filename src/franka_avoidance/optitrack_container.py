"""
Container which smoothenes position (and rotation) of incoming obstacles.
"""
import numpy as np

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.container import ObstacleContainer

from franka_avoidance.optitrack_interface import OptitrackInterface


class OptitrackContainer(ObstacleContainer):
    def __init__(self):
        obstacle_ids = []
        obstacle_offsets = []

        # Setup full optitrack callback
        self.optitrack_reciever = OptitrackInterface()

    def append(
        self,
        obstacle: Obstacle,
        obstacle_id: int,
        start_position: np.ndarray,
    ):
        super().append(obstacle)
        # TODO: create new topic each time
        if not hasattr(obstacle, "name"):
            raise ValueError("Topic name 'name' needed to recognize obstacle")

        pass

    def callback(self):
        pass
