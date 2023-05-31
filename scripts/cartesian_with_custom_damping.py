#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

# LASA / Control-Libraries
import state_representation as sr
from controllers import create_cartesian_controller, CONTROLLER_TYPE
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage

from vartools.linalg import get_orthogonal_basis

# Custom libraries
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface


class CartesianSpaceController(Node):
    def __init__(
        self,
        robot: RobotInterface,
        freq: float = 100.0,
        node_name="velocity_controller",
        is_simulation: bool = True,
    ) -> None:
        super().__init__(node_name)
        self.robot = robot
        self.rate = self.create_rate(freq)
        period = 1.0 / freq

        self.ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
        if is_simulation:
            print("Control mode: simulation")
            self.linear_principle_damping = 1.0
            self.linear_orthogonal_damping = 1.0
            self.angular_stiffness = 0.5
            self.angular_damping = 0.5

        else:
            print("Control mode: real")
            self.linear_principle_damping = 50.0
            self.linear_orthogonal_damping = 50.0
            self.angular_stiffness = 2.0
            self.angular_damping = 2.0

        self.ctrl.set_parameter_value(
            "linear_principle_damping",
            self.linear_principle_damping,
            sr.ParameterType.DOUBLE,
        )
        self.ctrl.set_parameter_value(
            "linear_orthogonal_damping",
            self.linear_orthogonal_damping,
            sr.ParameterType.DOUBLE,
        )
        self.ctrl.set_parameter_value(
            "angular_stiffness", self.angular_stiffness, sr.ParameterType.DOUBLE
        )
        self.ctrl.set_parameter_value(
            "angular_damping", self.angular_damping, sr.ParameterType.DOUBLE
        )

        self.ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds.set_parameter_value(
            "gain", [50.0, 50.0, 50.0, 20.0, 20.0, 20.0], sr.ParameterType.DOUBLE_ARRAY
        )

        self.clamp_linear = 0.25
        self.clamp_angular = 0.5

        # Get robot state to set up the target in the same frame
        while not (state := self.robot.get_state()) and rclpy.ok():
            print("Awaiting first state.")

        self.attractor_position = np.array([0.6, -0.2, 0.5])
        self.attractor_quaternion = np.array([0.0, 1.0, 0.0, 0.0])

        target = sr.CartesianPose(
            state.ee_state.get_name(),
            self.attractor_position,
            self.attractor_quaternion,
            state.ee_state.get_reference_frame(),
        )

        self.ds.set_parameter_value(
            "attractor",
            target,
            sr.ParameterType.STATE,
            sr.StateType.CARTESIAN_POSE,
        )

        self.create_controller_dissipative()

        self.timer = self.create_timer(period, self.controller_callback)

    # def run(self):
    #     while rclpy.ok():
    #         self.controller_callback()

    def create_controller_dissipative(self) -> None:
        """Simple dissipative controller to obtain the desired state.
        The state is assumed to have zero force / torque,
        as this will be transferred further."""
        # initialize controller : Fcomannd = I*x_dd + D*x_d + K*x
        self.ctrl_dissipative = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE)
        self.ctrl_dissipative.set_parameter_value(
            "stiffness", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )
        self.ctrl_dissipative.set_parameter_value(
            "inertia", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )

        D = np.diag(
            [
                self.linear_orthogonal_damping,
                self.linear_orthogonal_damping,
                self.linear_orthogonal_damping,
                self.angular_damping,
                self.angular_damping,
                self.angular_damping,
            ]
        )

        self.ctrl_dissipative.set_parameter_value("damping", D, sr.ParameterType.MATRIX)

    def update_dissipative_controller(self, desired_twist: sr.CartesianTwist) -> None:
        linear_velocity = desired_twist.get_linear_velocity()
        if not (lin_norm := np.linalg.norm(linear_velocity)):
            D = np.diag(
                [
                    self.linear_orthogonal_damping,
                    self.linear_orthogonal_damping,
                    self.linear_orthogonal_damping,
                    self.angular_damping,
                    self.angular_damping,
                    self.angular_damping,
                ]
            )
            self.ctrl_dissipative.set_parameter_value(
                "damping", D, sr.ParameterType.MATRIX
            )
            return

        D = np.zeros((6, 6))

        # Angular damping
        D[3:, 3:] = np.eye(3) * self.angular_damping

        # Linear damping
        D_linear = np.diag(
            [
                self.linear_principle_damping,
                self.linear_orthogonal_damping,
                self.linear_orthogonal_damping,
            ]
        )
        E = get_orthogonal_basis(linear_velocity / lin_norm)
        D[:3, :3] = E @ D_linear @ E.T
        self.ctrl_dissipative.set_parameter_value("damping", D, sr.ParameterType.MATRIX)

    def controller_callback(self) -> None:
        command = CommandMessage()
        command.control_type = [ControlType.EFFORT.value]

        state = self.robot.get_state()
        if not state:
            return

        # !!!Current force / torque are eliminated in order to not 'overcompensate'
        state.ee_state.set_force(np.zeros(3))
        state.ee_state.set_torque(np.zeros(3))

        desired_twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        desired_twist.clamp(self.clamp_linear, self.clamp_angular)

        # Update Damping-matrix based on desired velocity
        self.update_dissipative_controller(desired_twist)

        cmnd_dissipative = self.ctrl_dissipative.compute_command(
            desired_twist, state.ee_state, state.jacobian
        )

        # command_torques = sr.JointTorques(cmnd_dissipative)

        command.joint_state = state.joint_state
        command.joint_state.set_torques(cmnd_dissipative.get_torques())
        self.robot.send_command(command)


if (__name__) == "__main__":
    print("[INFO] Starting Cartesian Damping controller  ...")
    rclpy.init()
    robot_interface = RobotInterface("*:1601", "*:1602")

    controller = CartesianSpaceController(
        robot=robot_interface, freq=100, is_simulation=False
    )

    try:
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    controller.destroy_node()

    rclpy.shutdown()
