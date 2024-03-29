#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

import threading

# LASA Libraries
import state_representation as sr
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage

# Custom libraries
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface


class JointSpaceController(Node):
    def __init__(
        self,
        robot: RobotInterface,
        freq: float = 100,
        node_name: str = "velocity_controller",
        target: sr.CartesianPose = None,
    ) -> None:
        super().__init__(node_name)
        self.robot = robot
        self.rate = self.create_rate(freq)
        period = 1.0 / freq

        # Get robot state to set up the target in the same frame
        while not (state := self.robot.get_state()) and rclpy.ok():
            print("Awaiting first state.")

        if target is None:
            target = sr.CartesianPose(
                state.ee_state.get_name(),
                np.array([0.6, 0.3, 0.5]),
                np.array([0.0, 1.0, 0.0, 0.0]),
                state.ee_state.get_reference_frame(),
            )

        self.ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds.set_parameter_value(
            "gain", [20.0, 20.0, 50.0, 10.0, 10.0, 10.0], sr.ParameterType.DOUBLE_ARRAY
        )
        self.ds.set_parameter_value(
            "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
        )

        self.timer = self.create_timer(period, self.controller_callback)

        # Command message type which will be passed to robot
        self.command = CommandMessage()
        self.command.control_type = [ControlType.VELOCITY.value]

    def controller_callback(self) -> None:
        state = self.robot.get_state()
        print("Try again...")
        if not state:
            return

        twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        twist.clamp(0.25, 1.0)

        self.command.joint_state = state.joint_state
        self.command.joint_state.set_velocities(
            np.linalg.lstsq(state.jacobian.data(), twist.get_twist(), rcond=None)[0]
        )
        self.robot.send_command(self.command)


if __name__ == "__main__":
    print("Starting Joint Space ....")
    rclpy.init()
    # robot_interface = RobotInterface("*:1601", "*:1602")
    robot_interface = RobotInterface.from_id(17)

    # Spin in a separate thread
    controller = JointSpaceController(robot=robot_interface, freq=100)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()

    rclpy.shutdown()
