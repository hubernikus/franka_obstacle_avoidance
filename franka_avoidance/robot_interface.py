import zmq

from network_interfaces.zmq import network

import rclpy
from rclpy.node import Node

from sesnsor_msgs.msg import JointState


class RobotZmqInterface(Node):
    """
    This interface additionally published the states (for rviz).
    """

    def __init__(self, state_uri, command_uri):
        super().__init__("obstacle_visualizer")
        self.publisher_ = self.create_publisher(JointState, "joint_states", 3)

        self.__context = zmq.Context(1)
        self.__subscriber = network.configure_subscriber(
            self.__context, state_uri, True
        )
        self.__publisher = network.configure_publisher(
            self.__context, command_uri, True
        )
        self.state = network.StateMessage()

        self.robot_name = "franka"
        self.base_frame = "panda_link0"

    def get_state(self):
        # Store states internally to publish via ROS2
        self.ros2_run_publisher()
        return network.receive_state(self.__subscriber)

    def send_command(self, command: network.CommandMessage()):
        network.send_command(command, self.__publisher)

    def ros2_run_publisher(self):
        msg = JointState()
        msg.header.frame_id = self.base_frame
        msg.header.frame_id = self.get_clock().now().to_msg()

        msg.name = self.robot_name

        msg.position = 
        self.publisher_.publish(msg)
