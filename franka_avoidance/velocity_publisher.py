"""
Publish a Velocity (as Wrench) for use in RViz.
"""
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

# from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped


def get_orientation_from_direction(direction: np.ndarray) -> np.ndarray:
    if not (dir_norm := np.linalg.norm(direction)):
        return Rotation.from_quat([1.0, 0, 0, 0.0])
    null_vector = np.array([1.0, 0, 0])

    rot_vec = np.cross([1.0, 0, 0.0], direction / dir_norm)
    if not (rotvec_norm := np.linalg.norm(rot_vec)):
        return Rotation.from_quat([1.0, 0, 0, 0.0])

    rot_vec = rot_vec / rotvec_norm
    # theta = np.arcsin(rotvec_norm)
    # quat = np.hstack((rot_vec * np.cos(theta / 2.0), [np.sin(theta / 2.0)]))
    # return Rotation.from_quat(quat)

    theta = np.arccos(np.dot(null_vector, direction))
    return Rotation.from_rotvec(rot_vec / rotvec_norm * theta)


class VelocityPublisher(Node):
    def __init__(self, name: str, frame: str) -> None:
        self.frame = frame
        super().__init__(f"velocity_{name}_visualizer")
        self.publisher_ = self.create_publisher(PoseStamped, f"velocity/{name}", 5)

        self.msg = PoseStamped()
        self.msg.header.frame_id = self.frame

    def publish(self, position: np.ndarray, velocity: np.ndarray) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()

        # self.msg.wrench.x = velocity[0]
        # self.msg.wrench.y = velocity[1]
        # self.msg.wrench.z = velocity[2]

        self.msg.pose.position.x = position[0]
        self.msg.pose.position.y = position[1]
        self.msg.pose.position.z = position[2]

        orientation = get_orientation_from_direction(velocity)
        quat = orientation.as_quat()
        self.msg.pose.orientation.x = quat[0]
        self.msg.pose.orientation.y = quat[1]
        self.msg.pose.orientation.z = quat[2]
        self.msg.pose.orientation.w = quat[3]

        self.publisher_.publish(self.msg)
