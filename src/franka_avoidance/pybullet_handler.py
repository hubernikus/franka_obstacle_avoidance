import numpy as np

import pybullet as pb

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse


class PybulletHandler:
    def __init__(self, obstacles) -> None:
        physicsClient = pb.connect(pb.DIRECT)  # p.DIRECT for non-graphical version

        self.connection_mode = None
        if self.connection_mode is None:
            self._client = pb.connect(pb.SHARED_MEMORY)
            if self._client <= 0:
                # return
                # else:
                print("Doing new connection.")
                self.connection_mode = pb.DIRECT

                self._client = pb.connect(self.connection_mode)

        # pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        # pb.configureDebugVisualizer(lightPosition=[-10, 0, 100])

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
                basePosition=obs.position,
                # basePosition=obstacle.position,
                useMaximalCoordinates=True,
            )
            breakpoint()
        self.update()

    def update(self, obstacles: list[Obstacle]) -> None:
        for oo, obs in enumerate(obstacles):
            pb.resetBasePositionAndOrientation(
                self.pb_obstacle_ids[oo],
                obs.position,
                obs.orientation.as_quat().flatten(),
            )
