""" A simple wrapper to recieve / send optitrack ZMQ data. """

from dataclasses import dataclass

import struct
import zmq

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class RigidBody:
    obs_id: int

    position: np.ndarray
    rotation: Rotation


class OptitrackInterface:
    # Message length is part of the interface (1x int + 7x float)
    # Do NOT change these parameters
    msg_length = 36
    msg_structure = "iffffffff"

    def __init__(self, tcp_socket: str = "tcp://0.0.0.0:5511"):

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind(tcp_socket)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

        # Avoid infinite wait
        self.socket.setsockopt(zmq.LINGER, 0)

    def get_messages(self) -> list[RigidBody]:
        print("Collecting optitrack-data from zmq-server...")
        binary_data = self.socket.recv()

        bodies = []
        n_bodies = int(len(binary_data) / self.msg_length)
        for ii in range(n_bodies):
            subdata = binary_data[ii * self.msg_length : (ii + 1) * self.msg_length]

            body_array = np.array(struct.unpack(self.msg_structure, subdata))

            obs_id = body_array[0]
            position = body_array[2:5]
            rotation = Rotation.from_quat(
                [body_array[6], body_array[7], body_array[8], body_array[5]]
            )

            bodies.append(RigidBody(obs_id, position, rotation))

        return bodies
