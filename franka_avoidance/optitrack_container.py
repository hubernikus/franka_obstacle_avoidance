"""
Container which smoothenes position (and rotation) of incoming obstacles.
"""
import copy
import warnings

from typing import Protocol, Optional

# from enum import Enum, auto
import rclpy
from rclpy.node import Node

import numpy as np


from vartools.states import ObjectPose

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.containers import ObstacleContainer

from franka_avoidance.optitrack_interface import OptitrackInterface

from franka_avoidance.state_filters import PositionFilter, SimpleOrientationFilter


class VisualizationHandler(Protocol):
    """Visualization handler which allows updating / publishing the with corresponding
    visualization tool."""

    def update(self, obstacles: list[Obstacle], obstacle_ids: list[int]) -> None:
        ...


class OptitrackContainer(ObstacleContainer):
    # TODO: This OptitrackContainer could just be a handler which contains an obstacleContainer
    def __init__(
        self,
        visualization_handler: Optional[VisualizationHandler] = None,
        use_optitrack: bool = True,
        update_frequency: float = 100.0,
    ):
        super().__init__()
        self.obstacle_ids = []
        self.obstacle_offsets = []
        self.position_filters = []
        self.orientation_filters = []
        self.update_frequency = update_frequency

        # Setup full optitrack callback
        self.use_optitrack = use_optitrack
        if use_optitrack:
            self.optitrack_interface = OptitrackInterface()
        else:
            self.optitrack_interface = None

        self.visualization_handler = visualization_handler

    def update(self):
        """Update positions based on optitrack."""
        if self.optitrack_interface is not None:
            self.update_from_optitrack()

        if self.visualization_handler is not None:
            obstacle_ids = np.arange(len(self))
            self.visualization_handler.update(self, obstacle_ids)

    def append(
        self,
        obstacle: Obstacle,
        obstacle_id: "str" = None,
        # start_position: np.ndarray,
    ):
        super().append(obstacle)
        self.obstacle_ids.append(obstacle_id)
        self.obstacle_offsets.append(copy.deepcopy(obstacle.pose))
        self.position_filters.append(
            PositionFilter(
                update_frequency=self.update_frequency,
                initial_position=obstacle.pose.position,
            )
        )
        self.orientation_filters.append(
            SimpleOrientationFilter(
                update_frequency=self.update_frequency,
                initial_orientation=obstacle.pose.orientation,
            )
        )

    def update_from_optitrack(self) -> None:
        print("Updating obstacles.")
        # Set positions
        obs_optitrack = self.optitrack_interface.get_messages()
        for obs_oo in obs_optitrack:
            try:
                idx = self.obstacle_ids.index(obs_oo.obs_id)
            except ValueError:
                print(f"Not this time number {obs_oo.obs_id}")
                warnings.warn(
                    f"Obstacle with id {obs_oo.obs_id} not found in obstacle container."
                )
                continue

            # Update position
            self.position_filters[idx].run_once(obs_oo.position)
            self.orientation_filters[idx].run_once(obs_oo.rotation)

            # Update obstacle
            self[idx].pose.position = self.position_filters[
                idx
            ].position + self.orientation_filters[idx].rotation.apply(
                self.obstacle_offsets[idx].position
            )
            self[idx].pose.orientation = (
                self.orientation_filters[idx].rotation
                * self.obstacle_offsets[idx].orientation
            )

            # TODO: update velocity

            # Move into robot frame
            if self.optitrack_interface.robot_body is not None:
                # TODO: verify that this is really correct....
                print("Updating with respect to robot.")
                self[idx].pose.rotation = self[idx].pose.orientation * self.optitrack_interface.robot_body.rotation.inv()
                # self[idx].pose.position = self.optitrack_interface.robot_body.rotation.inv().apply(self[idx].pose.position) + \
                self[idx].pose.position = self.optitrack_interface.robot_body.rotation.apply(self[idx].pose.position) + \
                    self.optitrack_interface.robot_body.position

                self[idx].linear_velocity = self.optitrack_interface.robot_body.rotation.apply(self.position_filters[idx].velocity)

                self.visualization_handler.base_frame = "panda_link0"

                # TODO: update velocity to frame of reference

            print("Obstacle", np.round(self[idx].linear_velocity, 3))

    def shutdown(self):
        self.visualization_handler.remove_all_obstacles()
