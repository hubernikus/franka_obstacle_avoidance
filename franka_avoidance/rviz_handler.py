"""
Run the obstacles with following library.
"""
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
# from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse
from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid
from dynamic_obstacle_avoidance.containers import ObstacleContainer

from nonlinear_avoidance.multi_obstacle_container import MultiObstacleContainer


class RvizHandler(Node):
    def __init__(self, base_frame: str = "world"):
        super().__init__("obstacle_visualizer")

        self.publisher_ = self.create_publisher(
            MarkerArray, "obstacle_visualization", 3
        )

        # self.pose_publisher = self.create_publisher(PoseArray, "obstacle_velocities", 3)

        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.ii = 0

        self.marker_array = MarkerArray()
        self.base_frame = base_frame

        self.id_dict = {}
        # self.update(obstacles, obstacle_ids)

    @property
    def base_frame(self) -> str:
        return self._base_frame

    @base_frame.setter
    def base_frame(self, value: str) -> None:
        if not isinstance(value, str):
            raise ValueError("Wrong type")

        self._base_frame = value

        for marker in self.marker_array.markers:
            marker.header.frame_id = self._base_frame

    def create_marker_obstacle(
        self,
        obstacle: Obstacle,
        obstacle_id,
        color: Optional[list[float, float, float, float]] = None,
    ) -> Marker:
        if color is None:
            color = [0.8, 0.4, 0.4, 1.0]

        marker = Marker()
        marker.header.frame_id = self.base_frame
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
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        return marker

    def update_marker(self, pose, ii_marker):
        # Update / set position and orientation
        self.marker_array.markers[ii_marker].pose.position.x = pose.position[0]
        self.marker_array.markers[ii_marker].pose.position.y = pose.position[1]
        self.marker_array.markers[ii_marker].pose.position.z = pose.position[2]

        quat = pose.orientation.as_quat().flatten()

        self.marker_array.markers[ii_marker].pose.orientation.x = quat[0]
        self.marker_array.markers[ii_marker].pose.orientation.y = quat[1]
        self.marker_array.markers[ii_marker].pose.orientation.z = quat[2]
        self.marker_array.markers[ii_marker].pose.orientation.w = quat[3]

    def create_multi_obstacles(
        self,
        obstacle_container: MultiObstacleContainer,
        obstacle_ids: list[int],
        colors: Optional[list] = None,
    ):
        # Assumption of constanc obstacles
        for tt, tree in enumerate(obstacle_container):
            for cc, component in enumerate(tree):
                if colors is None:
                    color = None
                else:
                    color = colors[tt]

                id_obs = len(self.marker_array.markers)

                self.marker_array.markers.append(
                    self.create_marker_obstacle(component, id_obs, color)
                )
                self.id_dict[obstacle_ids[tt], cc] = id_obs
                # self.pose_array.poses.append(PoseStamped)
        # Do poses, too

    def remove_obstacles(self, obstacle_ids: list[int]):
        if True:
            raise NotImplementedError("Implemented updateing of the iterator .")

        for tt in obstacle_ids:
            cc = 0
            while True:
                try:
                    it_id = self.id_dict.pop(obstacle_ids[tt], cc)
                except KeyError:
                    print("Removed all obstacles of {tt}.")
                    break

                self.marker_array.markers.pop(it_id)
                cc += 1

    def update_multi_obstacle(
        self,
        obstacle_container: MultiObstacleContainer,
        obstacle_ids: list[int],
        colors=None,
    ):
        # Assumption of constanc obstacles
        for tt, tree in enumerate(obstacle_container):
            for cc, component in enumerate(tree):
                it_id = self.id_dict[obstacle_ids[tt], cc]

                self.update_marker(component.pose, it_id)

        self.publisher_.publish(self.marker_array)

    def update(
        self, obstacles: list[Obstacle], obstacle_ids: Optional[list] = None
    ) -> None:
        if obstacle_ids is None:
            obstacle_ids = np.arange(len(obstacles))

        # print("Doing rviz handle.")
        existing_ids = np.array([marker.id for marker in self.marker_array.markers])

        for obs, id_obs in zip(obstacles, obstacle_ids):
            indexes_marker = np.where(existing_ids == id_obs)[0]

            if not len(indexes_marker):
                self.marker_array.markers.append(
                    self.create_marker_obstacle(obs, id_obs)
                )
                ii_marker = -1
            else:
                # List -> int
                ii_marker = indexes_marker[0]

            self.update_marker(obs.pose, ii_marker)

        if len(obstacles) != len(self.marker_array.markers):
            raise NotImplementedError("Implemented removing of obstacles.")

        self.publisher_.publish(self.marker_array)

    def empty_obstacle_cache(self):
        self.marker_array = MarkerArray()

    def remove_all_obstacles(self):
        try:
            self.marker_array.markers = []
            self.publisher_.publish(self.marker_array)
        except rclpy._rclpy_pybind11.RCLError:
            print("ROScly-node already shutdown - no empty publishing possible.")
