from dataclasses import dataclass

import struct
import zmq
from network_interfaces.zmq import network

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class RigidBody:
    obs_id: int
    
    position: np.ndarray
    rotation: Rotation
    

class OptitrackInterface:
    # Message length is part of the interface (1x int + 7x float)
    # Do NOT change thi
    msg_length = 36
    msg_structure = 'iffffffff'
    
    def __init__():
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind("tcp://0.0.0.0:5511")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

    def get_message(self):
        print("Collecting optitrack-data from zmq-server...")
        
        bodies = []
        n_bodies = int(len(data) / self.msg_length)

        binary_data = self.socket.recv()
        for ii in range(n_bodies):
            subdata = data[ii * self.msg_length:(ii + 1) * self.msg_length]
            
            body_array = np.array(struct.unpack(self.msg_structure, subdata))
            
            obs_id = body_array[0]
            position = np.array(body_array[2:5]
            rotation = Rotation.from_quat([body_array[6],body_array[7],body_array[8],body_array[5]])

            self.body.append(RigidBody(obs_id, position, rotation))

        return self.bodies
