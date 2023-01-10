#!/usr/bin/env python3
import copy
import math

import numpy as np

import rclpy
from rclpy.node import Node

import threading

# LASA Libraries
import state_representation as sr
from controllers import create_cartesian_controller, CONTROLLER_TYPE
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage

# Custom libraries
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface

from vartools.linalg import get_orthogonal_basis


class CartesianSpaceController(Node):
    def __init__(self, robot, freq: float = 100, node_name="velocity_controller", is_simulation: bool = True):
        super().__init__(node_name)
        self.robot = robot
        self.rate = self.create_rate(freq)

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
            self.angular_stiffness = 0.0
            self.angular_damping = 0.0
            # self.angular_stiffness = 0.0
            # self.angular_damping = 0.0

        self.ctrl.set_parameter_value("linear_principle_damping", self.linear_principle_damping, sr.ParameterType.DOUBLE)
        self.ctrl.set_parameter_value("linear_orthogonal_damping", self.linear_orthogonal_damping, sr.ParameterType.DOUBLE)
        self.ctrl.set_parameter_value("angular_stiffness", self.angular_stiffness, sr.ParameterType.DOUBLE)
        self.ctrl.set_parameter_value("angular_damping", self.angular_damping, sr.ParameterType.DOUBLE)

        self.ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds.set_parameter_value(
            "gain", [50.0, 50.0, 50.0, 10.0, 10.0, 10.0], sr.ParameterType.DOUBLE_ARRAY
        )

        self.ds_dissipative = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds_dissipative.set_parameter_value(
            "gain", [50.0, 50.0, 50.0, 5.0, 5.0, 5.0], sr.ParameterType.DOUBLE_ARRAY
        )

        self.clamp_linear = 0.25
        self.clamp_angular = 0.5

        # Get robot state to set up the target in the same frame
        while not (state := self.robot.get_state()) and rclpy.ok():
            print("Awaiting first state.")

        self.attractor_quaternion = np.array([0.0, 1.0, 0.0, 0.0])
        target = sr.CartesianPose(
            state.ee_state.get_name(),
            np.array([0.6, -0.2, 0.5]),
            self.attractor_quaternion,
            state.ee_state.get_reference_frame(),
        )
        self.ds.set_parameter_value(
            "attractor",
            target,
            sr.ParameterType.STATE,
            sr.StateType.CARTESIAN_POSE,
        )

        self.ds_dissipative.set_parameter_value(
            "attractor",
            target,
            sr.ParameterType.STATE,
            sr.StateType.CARTESIAN_POSE,
        )

    def run(self):
        while rclpy.ok():
            self.controller_callback()

    def controller_compliant_twist(self, state):
        twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        twist.clamp(self.clamp_linear, self.clamp_angular)

        print()
        print("delta state - cmpl twist: \n", state.ee_state - twist)
        print()

        return self.ctrl.compute_command(
            twist, state.ee_state, state.jacobian)

    def controller_dissipative(self, state, desired_twist):
        ### DISSIPATIVE (LINEAR) ###
        # twist = sr.CartesianTwist(self.ds_dissipative.evaluate(state.ee_state))
        # twist.clamp(self.clamp_linear, self.clamp_angular * 0.5)
        if desired_twist is None:
            desired_twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
            desired_twist.clamp(self.clamp_linear, self.clamp_angular)

        # Move to relative velocity frame
        linear_velocity = desired_twist.get_linear_velocity()
        # angular_velocity = twist_dissipative.get_angular_velocity()
        angular_velocity = desired_twist.get_angular_velocity()

        # initialize controller : Fcomannd = I*x_dd + D*x_d + K*x
        ctrl_dissipative = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE)
        ctrl_dissipative.set_parameter_value(
            "damping", np.zeros((6, 6)), sr.ParameterType.MATRIX)

        ctrl_dissipative.set_parameter_value(
            "stiffness", np.zeros((6, 6)), sr.ParameterType.MATRIX)

        ctrl_dissipative.set_parameter_value(
            "inertia", np.zeros((6, 6)), sr.ParameterType.MATRIX)

        # Approach position - Damping linear velocity
        D = np.zeros((6, 6))
        D[:3, :3] = np.diag([
            self.linear_principle_damping,
            self.linear_orthogonal_damping,
            self.linear_orthogonal_damping
        ])

        E = get_orthogonal_basis(linear_velocity)
        # Sanity check
        if not np.allclose(E @ E.T, np.eye(3)):
            # Error -> non-orthogonal basis!!!
            breakpoint()

        D[:3, :3] = E @ D[:3, :3] @ E.T

        # Integrate velocity
        # D[3:, 3:] = np.eye(3) * (self.angular_damping + self.angular_stiffness)
        print(np.round(D, 3))
        ctrl_dissipative.set_parameter_value("damping", D, sr.ParameterType.MATRIX)

        # print(self.ctrl.get_parameters())
        # D_real = self.ctrl.get_parameter_value("damping")
        # print("D", D_real)

        des_state = sr.CartesianState(
            state.ee_state.get_name(),
            state.ee_state.get_reference_frame(),
        )

        print("des twist", desired_twist)
        print("state", state.ee_state.get_linear_velocity())
        print("state", state.ee_state.get_angular_velocity())

        des_state.set_linear_velocity(linear_velocity)
        des_state.set_angular_velocity(angular_velocity)

        print()
        print("delta state - passive: \n", state.ee_state - desired_twist)
        print()

        return ctrl_dissipative.compute_command(
            desired_twist, state.ee_state, state.jacobian
        )

    def controller_velocity_impedance(self, state, desired_twist=None):
        ### Velocity IMPEDANCE (ANGULAR) ###
        # Damp / reduce velocity [to zero...]
        if desired_twist is None:
            desired_twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
            desired_twist.clamp(self.clamp_linear, self.clamp_angular)

        ctrl_impedance = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE)
        K = np.zeros((6, 6))
        K[3:, 3:] = np.eye(3) * self.angular_stiffness
        ctrl_impedance.set_parameter_value(
            "stiffness", K, sr.ParameterType.MATRIX)

        D = np.zeros((6, 6))
        D[3:, 3:] = np.eye(3) * self.angular_damping
        # ctrl_impedance.set_parameter_value("stiffness", D, sr.ParameterType.MATRIX)
        # get_parameter_value("damping")
        # breakpoint()

        des_virt_state = sr.CartesianState(
            state.ee_state.get_name(),
            state.ee_state.get_reference_frame(),
        )

        angular_velocity = desired_twist.get_angular_velocity()
        if angular_norm := np.linalg.norm(angular_velocity):
            dt = 1
            theta = angular_norm * dt * 0.5
            q_w = math.cos(theta)
            q_vec = angular_velocity / angular_norm * math.sin(theta)

            des_virt_state.set_orientation(np.hstack((q_w, q_vec)))

        # des_virt_state.set_linear_velocity(twist.get_linear_velocity())
        des_virt_state.set_angular_velocity(desired_twist.get_angular_velocity())

        virt_state = sr.CartesianState(
            state.ee_state.get_name(),
            state.ee_state.get_reference_frame(),
        )
        virt_state.set_linear_velocity(np.zeros(3))
        virt_state.set_angular_velocity(np.zeros(3))

        return ctrl_impedance.compute_command(
            des_virt_state, virt_state, state.jacobian)

    def controller_callback(self) -> None:
        command = CommandMessage()
        command.control_type = [ControlType.EFFORT.value]

        state = self.robot.get_state()
        if not state:
            self.rate.sleep()
            return

        # print("State", state.ee_state.get_orientation())
        cmnd_compliant_twist = self.controller_compliant_twist(state)
        # print("Command CompliantTwist: ", cmnd_compliant_twist.get_torques())

        desired_twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        desired_twist.clamp(self.clamp_linear, self.clamp_angular)

        cmnd_dissipative = self.controller_dissipative(state, desired_twist)

        # print("Cmnd Dissipative", cmnd_dissipative)

        cmnd_impedance = self.controller_velocity_impedance(state, desired_twist)

        # tmp_torque = tmp_ctrl.compute_command(twist, state.ee_state, state.jacobian)
        print("T | Compliant", np.round(cmnd_compliant_twist.get_torques(), 3))
        print("T | Dissipat.", np.round(cmnd_dissipative.get_torques(), 3))
        print("T | Impedance", np.round(cmnd_impedance.get_torques(), 3))

        cmnd_torque = cmnd_dissipative + cmnd_impedance

        # command_torques = sr.JointTorques(cmnd_compliant_twist)
        # breakpoint()
        command_torques = sr.JointTorques(cmnd_compliant_twist)

        command.joint_state = state.joint_state
        command.joint_state.set_torques(command_torques.get_torques())

        # self.robot.send_command(command)
        print("Send another one.")
        self.rate.sleep()


if __name__ == "__main__":
    print("Starting Joint Space....")
    rclpy.init()
    # rospy.init_node("test", anonymous=True)
    robot_interface = RobotInterface("*:1601", "*:1602")

    # Spin in a separate thread
    controller = CartesianSpaceController(
        robot=robot_interface, freq=100, is_simulation=False)

    thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    thread.start()

    try:
        controller.run()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
