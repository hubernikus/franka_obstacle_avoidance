""" Repeats control command to ensure an update a constant, fast update rate"""
from typing import Optional
import signal
import time

import numpy as np

# ROS related
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Point

# Control library
import state_representation as sr

from franka_avoidance.controller_interface import ImpedanceCommandHandler
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface


class TwistRepeater(Node):
    """
    The TwistRepeater is created to ensure a fast udpate frequency even if the topic is slow.
    This ensures a smooth transition and constant update frequencey even with slow controller.
    BUT: it initiates a dealy of dt_transition, and dt_stop for STOPPING (!)"""

    def __init__(
        self,
        robot,
        freq_out: int = 1000,
        dt_transition: float = 0.01,
        dt_timeout: float = 0.15,
    ):
        super().__init__("Twist_repeater")
        self.robot = robot

        self.freq_out = freq_out
        self.loop_rate = 1.0 / self.freq_out
        self._loop_rate = self.create_rate(self.loop_rate, self.get_clock())

        self.dt_transition = dt_transition
        self.dt_timeout = dt_timeout

        self._last_msg_time = -1
        self._it_transition = 0
        self._n_transition = int(dt_transition * freq_out) + 1

        self.command_handler = ImpedanceCommandHandler.create_realworld()

        self.subscription = self.create_subscription(
            TwistStamped, "/twist_repeater", self.callback_twist, 5
        )
        self.subscription  # prevent unused variable warning

        # Create a PoseStamped publisher
        self.pose_publisher = self.create_publisher(PoseStamped, "/ee_pose", 5)

        self._old_twist = np.zeros(6)
        self._target_twist = np.zeros(6)
        self._current_twist = np.zeros(6)
        self._delta_twist = np.array(6)

        self.twist_command = sr.CartesianTwist()

        self._msg = TwistStamped()

        self.timer = self.create_timer(self.loop_rate, self.controller_callback)
        print("Finished initialization of reapeater.")

    @staticmethod
    def twist_to_array(twist: Twist) -> np.ndarray:
        return np.array(
            [
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ]
        )

    @staticmethod
    def array_to_twist(array: np.ndarray) -> Twist:
        twist = Twist()
        twist.linear.x = array[0]
        twist.linear.y = array[1]
        twist.linear.z = array[2]
        twist.angular.x = array[3]
        twist.angular.y = array[4]
        twist.angular.z = array[5]
        return twist

    @staticmethod
    def array_to_sr_twist(array: np.ndarray) -> sr.CartesianTwist:
        return sr.CartesianTwist("ee_twist", array[:3], array[3:])

    def callback_twist(self, msg: TwistStamped) -> None:
        self._it_transition = 0
        self._msg = msg

        self._last_msg_time = self.get_clock().now().nanoseconds * 1e-9

        self._old_twist = self._current_twist
        self._target_twist = self.twist_to_array(msg.twist)
        self._delta_twist = (self._target_twist - self._old_twist) / self._n_transition

    def ctrl_c_handler(self, signum, frame):
        print("\n  Control-C detected. Shutting down...")
        self.shutdown()
        raise KeyboardInterrupt

    def shutdown(self) -> None:
        # TODO: This still leads to weird behavior / unnexpected accelerations
        # Publish zero twist
        self.set_zero_twist()

        # Wait for transition to finish
        for _ in range(self._n_transition * 3):
            # Make sure to complete transition
            self.controller_callback()

            if np.allclose(self._current_twist, np.zeros(6)):
                print("Standing still.")
                break

            # Somehow ROS gives me an error...
            # Connetion is ZMQ only anyways
            # self._loop_rate.sleep()
            time.sleep(self.loop_rate)

        # Just to be sure - send it again
        for _ in range(10):
            self.controller_callback()
            time.sleep(self.loop_rate)

        self.destroy_node()
        print("Shutdown of node succesfull.")

    def set_zero_twist(self) -> None:
        self._it_transition = 0
        self._last_msg_time = self.get_clock().now().nanoseconds * 1e-9

        self._old_twist = self._current_twist
        self._target_twist = np.zeros(6)
        self._delta_twist = (self._target_twist - self._old_twist) / self._n_transition

    def controller_callback(self):
        # Check time delay of last message
        state = self.robot.get_state()
        if not state:
            return

        # Publish pose (since only one process can connect to zmq)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = state.ee_state.get_reference_frame()
        point = state.ee_state.get_position()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        quaternion = state.ee_state.get_orientation()
        pose.pose.orientation.x = quaternion.x
        pose.pose.orientation.y = quaternion.y
        pose.pose.orientation.z = quaternion.z
        pose.pose.orientation.w = quaternion.w
        self.pose_publisher.publish(pose)

        if (
            self.get_clock().now().nanoseconds * 1e-9 - self._last_msg_time
            > self.dt_timeout
        ):
            # No message received slowing down...
            # print(f"[INFO] No message received for {self.dt_timeout} seconds. Stopping")
            self.set_zero_twist()

        if self._it_transition < self._n_transition:
            self._it_transition += 1
            self._current_twist = (
                self._delta_twist * self._it_transition + self._old_twist
            )
            self.twist_command = self.array_to_sr_twist(self._current_twist)
            self.twist_command.set_reference_frame(state.ee_state.get_reference_frame())
            # print(f"Transitioning... [{self._it_transition}/{self._n_transition}]]")
            # print(self.twist_command)

        # print("twist", self.twist_command)
        self.command_handler.update_from_cartesian_twist(self.twist_command, state)
        # print("command", self.command_handler.get_command())
        self.robot.send_command(self.command_handler.get_command())


if __name__ == "__main__":
    print("Starting twist-control repeater...")

    rclpy.init()
    robot_interface = RobotInterface.from_id(17)

    repeater = TwistRepeater(robot=robot_interface)
    # Create ctrl-c handler
    signal.signal(signal.SIGINT, repeater.ctrl_c_handler)

    try:
        print("Starting controller.")
        rclpy.spin(repeater)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
