"""
Container which smoothenes position (and rotation) of incoming obstacles.
"""
import warnings

try:
    import pybullet as pb
except ModuleNotFoundError:
    warnings.warn("PyBullet not imported - no publishing possible.")

import numpy as np

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.container import ObstacleContainer

from franka_avoidance.optitrack_interface import OptitrackInterface


def pybullet_publisher(obstacles):
    self._pb_obstacle_ids = []
    # Goal Sphere
    for oo, obs in enumerate(obstacles):
        if isinstance(obs, "Ellipse"):
            # pb_obstacle_type = pb.GEOM_SPHERE
            pb_obstacle_type = pb.GEOM_ELLIPSOID
        else:
            raise NotImplementedError("Given obstacle type has not been defined.")

        self._pb_obstacle_ids.append(
            pb.createVisualShape(
                shapeType=pb_obstacle_type,
                halfExtents=obs.axes_length / 2.0,
                rgbaColor=[0.1, 0.9, 0.1, 0.9],
                specularColor=[0.4, 0.4, 0],
            )
        )
        pb_ellipse = pb.createMultiBody(
            baseMass=0,
            baseInertialFramePosition=[0, 0, 0],
            baseVisualShapeIndex=self._pb_obstacle_ids,
            # basePosition=obstacle.position,
            useMaximalCoordinates=True,
        )


def pybullet_initializer(obstacles):
    for oo in range(len(obstacles)):
        pb.resetBasePositionAndOrientation(
            self._pb_obstacle_ids[pp], obs.position, obs.orintation.as_quat()
        )


def rviz_publisher(obstacle):
    pass


def rviz_initializer(obstacle):
    pass


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
