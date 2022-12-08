"""
Container which smoothenes position (and rotation) of incoming obstacles.
"""
import warnings
from enum import Enum, auto


try:
    import pybullet as pb
except ModuleNotFoundError:
    warnings.warn("PyBullet not imported - no publishing possible.")

import numpy as np

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.containers import ObstacleContainer

from franka_avoidance.optitrack_interface import OptitrackInterface


class PybulletHandler:
    def __init__(self, obstacles) -> None:
        physicsClient = pb.connect(pb.DIRECT)  # p.DIRECT for non-graphical version

        self.pb_obstacle_ids = []
        # Goal Sphere
        for oo, obs in enumerate(obstacles):
            if isinstance(obs, Ellipse):
                # pb_obstacle_type = pb.GEOM_ELLIPSOID
                pb_obstacle_type = pb.GEOM_SPHERE

                self.pb_obstacle_ids.append(
                    pb.createVisualShape(
                        shapeType=pb_obstacle_type,
                        # halfExtents=obs.axes_length / 2.0,
                        radius=1.0,
                        rgbaColor=[0.1, 0.9, 0.1, 0.9],
                        specularColor=[0.4, 0.4, 0],
                    )
                )
            else:
                raise NotImplementedError("Given obstacle type has not been defined.")

            pb_obstacle = pb.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseVisualShapeIndex=self.pb_obstacle_ids[oo],
                # basePosition=obstacle.position,
                useMaximalCoordinates=True,
            )

    def update(self, obstacles: list[Obstacle]) -> None:
        for oo, obs in enumerate(obstacles):
            pb.resetBasePositionAndOrientation(
                self.pb_obstacle_ids[oo], obs.position, obs.orientation.as_quat()
            )


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

    def publish_obstacles(self):
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
            center_position=np.zeros([0, 0, 0]),
            axes_length=np.array([1, 1, 1])
            # orientation
            # name="",
        ),
        obstacle_id=0,
    )

    obstacles.visualization_handler = PybulletHandler(obstacles)
    obstacles.publish_obstacles()
