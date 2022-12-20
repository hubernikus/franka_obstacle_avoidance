import zmq

from network_interfaces.zmq import network

from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotZmqInterface(Node):
    """
    This interface additionally published the states (for rviz).
    """

    def __init__(self, state_uri, command_uri, do_ros_publish: bool = True):
        self.do_ros_publish = do_ros_publish
        if self.do_ros_publish:
            print(f"Creating ZMQ Node")
            super().__init__("robot_zmq_interface")
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
        state = network.receive_state(self.__subscriber)
        if state is None:
            print("RobotZMQ: No state recieved.")
            return

        if self.do_ros_publish:
            self.ros2_run_publisher(state)
        return state

    def send_command(self, command: network.CommandMessage()):
        network.send_command(command, self.__publisher)

    def ros2_run_publisher(self, state):
        msg = JointState()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = state.joint_state.get_names()
        msg.position = state.joint_state.get_positions().tolist()
        msg.velocity = state.joint_state.get_velocities().tolist()
        msg.effort = state.joint_state.get_torques().tolist()

        self.publisher_.publish(msg)
