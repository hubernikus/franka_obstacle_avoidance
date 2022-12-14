"""
python src/franka_avoidance/optitrack_container.py
to run example file
"""
import argparse

import warnings

import numpy as np

import pybullet as pb
import pybullet_data
from pybullet_zmq.simulator import PyBulletZmqWrapper

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid


class ZmqHandler:
    def __init__(self):
        parser = argparse.ArgumentParser(
            description="Start PyBullet simulation with ZMQ interface."
        )
        parser.add_argument(
            "-c",
            "--config-file",
            type=str,
            default="/home/ros2/pybullet_zmq/pybullet_zmq/config/franka_config.yaml",
            help="Configuration file for the simulation (default: franka_config.yaml)",
        )

        args = parser.parse_args()

        self.wrapper = PyBulletZmqWrapper(args.config_file)
        self.wrapper.start_pybullet_zmq_wrapper()


class PybulletHandler:
    # Define id-domain to avoid overlay with robot(?)
    id_domain = 2000

    @property
    def pb(self):
        if self._pb is None:
            return self.zmq_handler.wrapper._pb
        else:
            return self._pb

    @pb.setter
    def pb(self, value):
        self._pb = value

    def error_patch(self):
        # Create tiny objects far away
        breakpoint()
        n_obs = 0
        for ii in range(n_obs):
            pb.createVisualShape(
                shapeType=pb.GEOM_SPHERE,
                radius=0.01,
                rgbaColor=[0.3, 0.3, 0.3, 0.9],
                specularColor=[0.4, 0.4, 0],
                physicsClientId=self._client,
            )
            pb.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseVisualShapeIndex=id_visual,
                basePosition=[100, 0, 0],
                physicsClientId=self._client,
            )

    def __init__(self, obstacles, obstacle_ids: list = None) -> None:
        self.connection_mode = None
        if self.connection_mode is None:
            self._client = pb.connect(pb.SHARED_MEMORY)
            if self._client < 0:
                print("[INFO] Doing new connection.")
                self.connection_mode = pb.DIRECT

                self._client = pb.connect(self.connection_mode)
            else:
                print("[INFO] Succesfully connected to existing pybullet simulation.")

        # pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        # pb.configureDebugVisualizer(lightPosition=[-10, 0, 100])

        pb.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.pb_visual_ids = []
        self.pb_body_ids = []

        for oo, obs in enumerate(obstacles):
            if isinstance(obs, Ellipse):
                pb_obstacle_type = pb.GEOM_SPHERE

                if obs.axes_length[0] != obs.axes_length[1] != obs.axes_length[2]:
                    warnings.warn("Ellipse is not a sphere. We take the longest axes.")

                radius = np.max(obs.axes_length)

                self.pb_visual_ids.append(
                    pb.createVisualShape(
                        shapeType=pb_obstacle_type,
                        radius=radius,
                        # halfExtents=(obs.axes_length / 2.0).tolist(),
                        # meshScale=(obs.axes_length / 2.0).tolist(),
                        rgbaColor=[0.3, 0.3, 0.3, 0.9],
                        specularColor=[0.4, 0.4, 0],
                        physicsClientId=self._client,
                    )
                )

            elif isinstance(obs, Cuboid):
                raise NotImplementedError("TODO: implement cuboids (check for 3D use).")
            else:
                raise NotImplementedError("Given obstacle type has not been defined.")

            self.delta_id_patch = 0
            self.pb_body_ids.append(
                pb.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseVisualShapeIndex=self.pb_visual_ids[oo] + self.delta_id_patch,
                    basePosition=obs.position,
                    # useMaximalCoordinates=True,
                    physicsClientId=self._client,
                )
            )

    def update(self, obstacles: list[Obstacle], obstacle_ids: list = None) -> None:
        for oo, obs in enumerate(obstacles):
            pb.resetBasePositionAndOrientation(
                self.pb_body_ids[oo],
                obs.position,
                obs.orientation.as_quat().flatten(),
                physicsClientId=self._client,
            )

    def remove_all_obstacles(self) -> None:
        # TODO: this currently does not work as it removes first one of the panda links..
        for obs_id in self.pb_body_ids:
            print(f"[INFO] Removing body #{obs_id}")
            pb.removeBody(obs_id, physicsClientId=self._client)
