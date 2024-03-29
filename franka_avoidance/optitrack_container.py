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
        fixed_robot_position: bool = True,
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

        self.fixed_robot_position = fixed_robot_position

        self.visualization_handler = visualization_handler

    def update(self, visualize=True):
        """Update positions based on optitrack."""
        if self.optitrack_interface is not None:
            self.update_from_optitrack()

        if self.visualization_handler is not None and visualize:
            obstacle_ids = np.arange(len(self))
            self.visualization_handler.update(self, obstacle_ids)

        # breakpoint()

    def publish_to_rviz(self):
        obstacle_ids = np.arange(len(self))
        self.visualization_handler.update(self, obstacle_ids)

    def append(
        self,
        obstacle: Obstacle,
        obstacle_id: Optional[int] = None,
        parent_id: Optional[Obstacle] = None,
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

            if self.fixed_robot_position:
                # Callibration is done such that the robot position is fixed (!) [the offset is hard coded]
                # robot_position = np.array([0.0, -0.43, 0.0])
                robot_position = np.array([0.0, 0.2, 0.0])
                self[idx].pose.position = self[idx].pose.position - robot_position
                self[idx].linear_velocity = self.position_filters[idx].velocity

            elif self.optitrack_interface.robot_body is not None:
                # TODO: verify that this is really correct....
                print("Updating with respect to robot.")
                self[idx].pose.rotation = (
                    self[idx].pose.orientation
                    * self.optitrack_interface.robot_body.rotation.inv()
                )
                # self[idx].pose.position = self.optitrack_interface.robot_body.rotation.inv().apply(self[idx].pose.position) + \
                # breakpoint()
                self[idx].pose.position = (
                    self.optitrack_interface.robot_body.rotation.apply(
                        self[idx].pose.position
                    )
                    - self.optitrack_interface.robot_body.position
                )

                self[
                    idx
                ].linear_velocity = self.optitrack_interface.robot_body.rotation.apply(
                    self.position_filters[idx].velocity
                )

                # self.visualization_handler.base_frame = "panda_link0"
                # Make sure pose of children is correctly updated for 'multi-obstacles'
                # TODO: update velocity to frame of reference
                # print('opti vel', self.position_filters[idx].velocity)

            if hasattr(self[idx], "update_pose"):
                self[idx].update_pose(self[idx].pose)

            # print("Obstacle", np.round(self[idx].linear_velocity, 3))

    def shutdown(self):
        self.visualization_handler.remove_all_obstacles()
