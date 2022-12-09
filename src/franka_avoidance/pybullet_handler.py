import numpy as np

import pybullet as pb

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse


class PybulletHandler:
    def __init__(self, obstacles) -> None:
        physicsClient = pb.connect(pb.DIRECT)  # p.DIRECT for non-graphical version

        self.connection_mode = None
        if self.connection_mode is None:
            self._client = pybullet.connect(pybullet.SHARED_MEMORY)
            breakpoint()
            if self._client >= 0:
                return
            else:
                self.connection_mode = pybullet.DIRECT

        self._client = pybullet.connect(self.connection_mode)

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
