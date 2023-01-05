""" A simple wrapper to recieve / send optitrack ZMQ data. """

from dataclasses import dataclass

import struct
import zmq

import numpy as np
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped


@dataclass
class RigidBody:
    obs_id: int

    position: np.ndarray
    rotation: Rotation


class OptitrackInterface(Node):
    """
    Properties
    ----------
    robot_id:
    """

    # Message length is part of the interface (1x int + 7x float)
    # Do NOT change these parameters
    msg_length = 36
    msg_structure = "iffffffff"

    def __init__(
        self,
        tcp_socket: str = "tcp://0.0.0.0:5511",
        robot_id: int = 16,
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

        self.robot_body = None

        # Avoid infinite wait
        # self.socket.setsockopt(zmq.LINGER, 0)

    def get_messages(self) -> list[RigidBody]:
        # print("[optitrack_interface] Collecting optitrack-data from zmq-server...")
        binary_data = self.socket.recv()

        bodies = []
        n_bodies = int(len(binary_data) / self.msg_length)

        for ii in range(n_bodies):
            # print(f"n bodies {n_bodies}")
            subdata = binary_data[ii * self.msg_length: (ii + 1) * self.msg_length]

            body_array = np.array(struct.unpack(self.msg_structure, subdata))
            obs_id = body_array[0]
            position = body_array[2:5]
            rotation = Rotation.from_quat(
                [body_array[6], body_array[7], body_array[8], body_array[5]]
            )

            if obs_id == self.robot_id:
                # print("got robot.")
                self.publish_roboot_transform(position, rotation)

                self.robot_body = RigidBody(self.robot_id, position, rotation)
                continue

            # print(f"Updating body: {obs_id}")

            bodies.append(RigidBody(obs_id, position, rotation))

        return bodies

    def publish_roboot_transform(
        self, position: np.ndarray, rotation: Rotation
    ) -> None:
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

        self.tf_broadcaster.sendTransform(tt)
