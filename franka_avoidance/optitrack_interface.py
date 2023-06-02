""" A simple wrapper to recieve / send optitrack ZMQ data. """

from dataclasses import dataclass
import warnings

import struct
import zmq

import numpy as np
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped


@dataclass
class RigidBody:
    """Optitrack RigidBody as recieved from the interace."""

    obs_id: int
    position: np.ndarray
    rotation: Rotation


@dataclass
class Pose3D:
    # TODO: this could be replaced with vartools.ObjectPose
    position: np.ndarray
    rotation: Rotation

    def transform_position_to_relative(self, position: np.ndarray) -> np.ndarray:
        new_position = self.rotation.apply(position) - self.position
        return new_position

    def transform_orientation_to_relative(self, rotation: Rotation) -> Rotation:
        new_rotation = rotation * self.rotation.inv()
        return new_rotation

    def transform_linear_velocity_to_relative(self, velocity: np.ndarray) -> np.ndarray:
        new_velocity = self.rotation.apply(velocity)
        return new_velocity


class SimpleRobot(Node):
    def __init__(
        self,
        robot_id: int = 17,
        robot_base_frame: str = "panda_link0",
    ):
        super().__init__("robot_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.optitrack_id = robot_id
        self.pose = Pose3D(position=np.zeros(3), rotation=Rotation.from_euler("x", 0))
        self.robot_base_frame = robot_base_frame

    def publish_robot_transform(self) -> None:
        tt = TransformStamped()

        tt.header.stamp = self.get_clock().now().to_msg()
        tt.header.frame_id = "world"

        tt.child_frame_id = self.robot_base_frame
        tt.transform.translation.x = self.pose.position[0]
        tt.transform.translation.y = self.pose.position[1]
        tt.transform.translation.z = self.pose.position[2]

        quat = self.pose.rotation.as_quat()
        tt.transform.rotation.x = quat[0]
        tt.transform.rotation.y = quat[1]
        tt.transform.rotation.z = quat[2]
        tt.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tt)


class OptitrackInterface(Node):
    """
    Properties
    ----------
    robot_id:
    """

    # Message length is part of the interface (1x int + 7x float)
    # Do NOT change these parameters
    MSG_LENGTH = 36
    MSG_STRUCTURE = "iffffffff"

    def __init__(
        self,
        tcp_socket: str = "tcp://0.0.0.0:5511",
        robot_id: int = 17,
        robot_base_frame: str = "panda_link0",
    ):
        if len(robot_base_frame):
            print(f"Creating Optitrack Node")
            super().__init__("optitrack_zmq_node")
            self.tf_broadcaster = TransformBroadcaster(self)

            self.robot_id = robot_id
            self.robot_base_frame = robot_base_frame
        else:
            # (Hopefully) not optitrack conform
            self.robot_id == -1

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind(tcp_socket)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        # self.socket.settimeout(1.0)
        # TODO: remove robot from here...
        self.robot_body = None

        # Avoid infinite wait
        # self.socket.setsockopt(zmq.LINGER, 0)

    def get_messages(self) -> list[RigidBody]:
        # print("[optitrack_interface] Collecting optitrack-data from zmq-server...")

        try:
            binary_data = self.socket.recv(flags=zmq.NOBLOCK)
        except zmq.error.Again:
            warnings.warn("No Optitrack recieved.")
            # breakpoint()
            return []

        bodies = []
        n_bodies = int(len(binary_data) / self.MSG_LENGTH)

        print(f"Got {n_bodies} bodies")
        for ii in range(n_bodies):
            subdata = binary_data[ii * self.MSG_LENGTH : (ii + 1) * self.MSG_LENGTH]

            body_array = np.array(struct.unpack(self.MSG_STRUCTURE, subdata))
            obs_id = int(body_array[0])
            position = body_array[2:5]
            rotation = Rotation.from_quat(
                [body_array[6], body_array[7], body_array[8], body_array[5]]
            )

            if obs_id == self.robot_id:
                self.publish_robot_transform(position, rotation)

                self.robot_body = RigidBody(self.robot_id, position, rotation)
                continue

            bodies.append(RigidBody(obs_id, position, rotation))

        # breakpoint()
        return bodies

    def publish_robot_transform(self, position: np.ndarray, rotation: Rotation) -> None:
        # Depreciated -> use SimpleRobot instead
        tt = TransformStamped()

        tt.header.stamp = self.get_clock().now().to_msg()
        tt.header.frame_id = "world"
        tt.child_frame_id = self.robot_base_frame
        tt.transform.translation.x = position[0]
        tt.transform.translation.y = position[1]
        tt.transform.translation.z = position[2]

        quat = rotation.as_quat()
        tt.transform.rotation.x = quat[0]
        tt.transform.rotation.y = quat[1]
        tt.transform.rotation.z = quat[2]
        tt.transform.rotation.w = quat[3]

        # print("[INFO] Published robot transform.")
        self.tf_broadcaster.sendTransform(tt)
