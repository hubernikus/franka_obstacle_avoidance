"""
Run the obstacles with following library.
"""
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid
from dynamic_obstacle_avoidance.containers import ObstacleContainer


class RvizHandler(Node):
    def __init__(
        self, obstacles: Optional[ObstacleContainer] = None, base_frame: str = "world"
    ):
        super().__init__("obstacle_visualizer")

        self.publisher_ = self.create_publisher(
            MarkerArray, "obstacle_visualization", 3
        )

        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.ii = 0

        self._base_frame = base_frame
        self.marker_array = MarkerArray()

        # self.update(obstacles, obstacle_ids)

    @property
    def base_frame(self) -> "str":
        return self._base_frame

    @base_frame.setter
    def base_frame(self, value: "str") -> None:
        self._base_frame = value

        for marker in self.marker_array.markers:
            marker.header.frame_id = self._base_frame

    def get_marker_obstacle(self, obstacle: Obstacle, obstacle_id) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        # breakpoint()
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "obstacles"
        marker.id = int(obstacle_id)
        if isinstance(obstacle, Ellipse):
            marker.type = Marker.SPHERE

        elif isinstance(obstacle, Cuboid):
            marker.type = Marker.CUBE

        marker.action = Marker.ADD

        # Scale
        marker.scale.x = obstacle.axes_length[0]
        marker.scale.y = obstacle.axes_length[1]
        marker.scale.z = obstacle.axes_length[2]

        # Color
        marker.color.a = 1.0
        marker.color.r = 0.8
        marker.color.g = 0.4
        marker.color.b = 0.4

        return marker

    def update(
        self, obstacles: list[Obstacle], obstacle_ids: Optional[list] = None
    ) -> None:
        if obstacle_ids is None:
            obstacle_ids = np.arange(len(obstacles))

        # print("Doing rviz handle.")
        existing_ids = np.array([marker.id for marker in self.marker_array.markers])

        # breakpoint()
        for obs, id_obs in zip(obstacles, obstacle_ids):
            indexes_marker = np.where(existing_ids == id_obs)[0]
            if not len(indexes_marker):
                self.marker_array.markers.append(self.get_marker_obstacle(obs, id_obs))
                ii_marker = -1
            else:
                # List -> int
                ii_marker = indexes_marker[0]

            # Update / set position and orientation
            self.marker_array.markers[ii_marker].pose.position.x = obs.position[0]
            self.marker_array.markers[ii_marker].pose.position.y = obs.position[1]
            self.marker_array.markers[ii_marker].pose.position.z = obs.position[2]

            quat = obs.orientation.as_quat().flatten()
            self.marker_array.markers[ii_marker].pose.orientation.x = quat[0]
            self.marker_array.markers[ii_marker].pose.orientation.y = quat[1]
            self.marker_array.markers[ii_marker].pose.orientation.z = quat[2]
            self.marker_array.markers[ii_marker].pose.orientation.w = quat[3]

        if len(obstacles) != len(self.marker_array.markers):
            raise NotImplementedError("Implemented removing of obstacles.")

        self.publisher_.publish(self.marker_array)

    def remove_all_obstacles(self):
        try:
            self.marker_array.markers = []
            self.publisher_.publish(self.marker_array)
        except rclpy._rclpy_pybind11.RCLError:
            print("ROScly-node already shutdown - no empty publishing possible.")
