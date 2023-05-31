"""
Publish a Velocity (as Wrench) for use in RViz.
"""
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class TrajectoryPublisher(Node):
    dimension = 3

    def __init__(self, functor: Callable, name: str, frame_id: str = "world") -> None:
        self.frame_id = frame_id
        super().__init__(f"trajectory_{name}_visualizer")
        self._publisher = self.create_publisher(Path, f"path/{name}", 5)
        self.functor = functor

        self.msg = Path()
        self.msg.header.frame_id = self.frame_id

        self.it_max = 15
        self.delta_time = 0.05
        self.atol_conv = 1e-3

    def publish(self, position: np.ndarray) -> np.ndarray:
        trajectory = np.zeros((self.dimension, self.it_max + 1))
        trajectory[:, 0] = position

        for ii in range(self.it_max):
            velocity = self.functor(position=trajectory[:, ii])
            if np.linalg.norm(velocity) < self.atol_conv:
                trajectory = trajectory[:, : ii + 1]
                break

            trajectory[:, ii + 1] = velocity * self.delta_time + trajectory[:, ii]

            if not (ii + 1) % (self.it_max // 2):
                print(f"Preparation Loop {ii + 1} / {self.it_max}")

        stamp = self.get_clock().now().to_msg()
        self.msg.header.stamp = stamp
        self.msg.poses: list[PoseStamped] = []
        for ii in range(trajectory.shape[1]):
            self.msg.poses.append(PoseStamped())
            self.msg.poses[ii].pose.position.x = trajectory[0, ii]
            self.msg.poses[ii].pose.position.y = trajectory[1, ii]
            self.msg.poses[ii].pose.position.z = trajectory[2, ii]
            # self.msg.poses[ii].pose.orientation.x = 0.0
            # self.msg.poses[ii].pose.orientation.y = 0.0
            # self.msg.poses[ii].pose.orientation.z = 0.0
            # self.msg.poses[ii].pose.orientation.w = 0.0
            self.msg.poses[ii].header.frame_id = self.frame_id
            self.msg.poses[ii].header.stamp = stamp

        self.msg.header.frame_id = self.frame_id
        self.msg.header.stamp = stamp

        # Publish Trajectory
        self._publisher.publish(self.msg)

        return trajectory
