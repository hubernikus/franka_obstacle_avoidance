#!/usr/bin/env python3
from typing import Optional
import threading

import numpy as np

import rclpy
from rclpy.node import Node

# LASA Libraries
import state_representation as sr
from controllers import create_cartesian_controller, CONTROLLER_TYPE
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage

# Custom libraries
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface


class CartesianSpaceController(Node):
    def __init__(
        self,
        robot,
        freq: float = 100,
        node_name="velocity_controller",
        is_simulation: bool = True,
        target: Optional[sr.CartesianPose] = None,
    ):
        super().__init__(node_name)
        self.robot = robot
        self.rate = self.create_rate(freq)
        period = 1.0 / freq

        self.ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
        if is_simulation:
            self.ctrl.set_parameter_value(
                "linear_principle_damping", 1.0, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "linear_orthogonal_damping", 1.0, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "angular_stiffness", 0.5, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "angular_damping", 0.5, sr.ParameterType.DOUBLE
            )
        else:
            self.ctrl.set_parameter_value(
                "linear_principle_damping", 50.0, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "linear_orthogonal_damping", 50.0, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "angular_stiffness", 2.0, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "angular_damping", 2.0, sr.ParameterType.DOUBLE
            )

        # Geut robot state to set up the target in the same frame
        while not (state := self.robot.get_state()) and rclpy.ok():
            print("Awaiting first state.")

        if target is None:
            quat = np.array([0.0, 1.0, 0.0, 0.0])
            target = sr.CartesianPose(
                state.ee_state.get_name(),
                np.array([0.6, -0.3, 0.5]),
                quat,
                state.ee_state.get_reference_frame(),
            )

        self.ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds.set_parameter_value(
            "gain", [50.0, 50.0, 50.0, 10.0, 10.0, 10.0], sr.ParameterType.DOUBLE_ARRAY
        )
        self.ds.set_parameter_value(
            "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
        )

        self.command = CommandMessage()
        self.command.control_type = [ControlType.EFFORT.value]

        self.timer = self.create_timer(period, self.controller_callback)

    def controller_callback(self) -> None:
        state = self.robot.get_state()

        if not state:
            return

        twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        twist.clamp(0.25, 0.5)

        # Compute the torques
        cartesian_command = self.ctrl.compute_command(
            twist, state.ee_state, state.jacobian
        )

        self.command.joint_state = state.joint_state

        # self.command_torques = sr.JointTorques(cartesian_command)
        # self.command.joint_state.set_torques(self.command_torques.get_torques())
        # print("torques", cartesian_command.get_torques())
        self.command.joint_state.set_torques(cartesian_command.get_torques())

        self.robot.send_command(self.command)


if __name__ == "__main__":
    print("Starting CartesianTwist controller ...")
    rclpy.init()
    # rospy.init_node("test", anonymous=True)
    robot_interface = RobotInterface("*:1601", "*:1602")

    # Spin in a separate thread
    controller = CartesianSpaceController(
        robot=robot_interface, freq=50, is_simulation=False
    )

    # thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    # thread.start()

    try:
        # controller.run()
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    # thread.join()
