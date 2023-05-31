#!/usr/bin/env python3
import copy
import math

import numpy as np
import scipy
from scipy.spatial.transform import Rotation as Rotation

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


def test_compare_controllers(self):
    lpd = 10.0
    lod = 9.0
    ang_stiff = 1.0
    ang_damping = 1.0
    compliant_twist = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
    compliant_twist.set_parameter_value(
        "linear_principle_damping", lpd, sr.ParameterType.DOUBLE
    )
    compliant_twist.set_parameter_value(
        "linear_orthogonal_damping", lod, sr.ParameterType.DOUBLE
    )
    compliant_twist.set_parameter_value(
        "angular_stiffness", ang_stiff, sr.ParameterType.DOUBLE
    )
    compliant_twist.set_parameter_value(
        "angular_damping", ang_damping, sr.ParameterType.DOUBLE
    )

    velocity_impedance = create_cartesian_controller(CONTROLLER_TYPE.VELOCITY_IMPEDANCE)
    K = np.zeros((6, 6))
    K[-3:, -3:] = np.eye(3) * ang_stiff
    velocity_impedance.set_parameter_value("stiffness", K, sr.ParameterType.MATRIX)
    D = np.zeros((6, 6))
    D[-3:, -3:] = np.eye(3) * ang_damping
    velocity_impedance.set_parameter_value("damping", D, sr.ParameterType.MATRIX)

    damping_eigenvalues = np.array([lpd, lod, lod, 0, 0, 0])
    dissipative = create_cartesian_controller(CONTROLLER_TYPE.DISSIPATIVE_LINEAR)
    dissipative.set_parameter_value(
        "damping_eigenvalues", damping_eigenvalues, sr.ParameterType.VECTOR
    )

    desired_twist = sr.CartesianTwist().Random("test")
    current_state = sr.CartesianState().Random("test")

    diagonal_eigenvalues = np.diag(damping_eigenvalues)
    basis = self.compute_orthogonal_basis(desired_twist.get_linear_velocity())
    D = basis * diagonal_eigenvalues * basis.transpose()
    print(D)

    compliant_command = compliant_twist.compute_command(desired_twist, current_state)
    impedance_command = velocity_impedance.compute_command(desired_twist, current_state)
    dissipative_command = dissipative.compute_command(desired_twist, current_state)
    print(dissipative.get_parameter_value("damping"))
    np.testing.assert_almost_equal(
        compliant_command.get_torque(), impedance_command.get_torque()
    )
    np.testing.assert_almost_equal(
        compliant_command.get_force(), dissipative_command.get_force()
    )


class CartesianSpaceController(Node):
    def __init__(
        self,
        robot,
        freq: float = 100,
        node_name="velocity_controller",
        is_simulation: bool = True,
    ):
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
            "gain", [50.0, 50.0, 50.0, 10.0, 10.0, 10.0], sr.ParameterType.DOUBLE_ARRAY
        )

        self.ds_dissipative = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds_dissipative.set_parameter_value(
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

        self.ds_dissipative.set_parameter_value(
            "attractor",
            target,
            sr.ParameterType.STATE,
            sr.StateType.CARTESIAN_POSE,
        )

        self.timer = self.create_timer(period, self.controller_callback)

    def run(self):
        while rclpy.ok():
            self.controller_callback()

    def controller_compliant_twist(self, state):
        twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        twist.clamp(self.clamp_linear, self.clamp_angular)

        # For this controller this should not be needed (I think..)
        tmp_state = copy.deepcopy(state.ee_state)
        tmp_state.set_force(0, 0, 0)
        tmp_state.set_torque(0, 0, 0)

        return self.ctrl.compute_command(twist, tmp_state, state.jacobian)

    def controller_dissipative(self, state, desired_twist):
        """Simple dissipative controller to obtain the desired state.
        The state is assumed to have zero force / torque, as this will be transferred further."""
        # initialize controller : Fcomannd = I*x_dd + D*x_d + K*x
        ctrl_dissipative = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE)
        ctrl_dissipative.set_parameter_value(
            "stiffness", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )
        ctrl_dissipative.set_parameter_value(
            "inertia", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )

        D = np.diag(
            [
                self.linear_principle_damping,
                self.linear_orthogonal_damping,
                self.linear_orthogonal_damping,
                self.angular_damping,
                self.angular_damping,
                self.angular_damping,
            ]
        )

        ctrl_dissipative.set_parameter_value("damping", D, sr.ParameterType.MATRIX)

        return ctrl_dissipative.compute_command(
            desired_twist, state.ee_state, state.jacobian
        )

    def controller_dissipative_linear(self, state, desired_twist):
        ### DISSIPATIVE (LINEAR) ###
        if desired_twist is None:
            desired_twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
            desired_twist.clamp(self.clamp_linear, self.clamp_angular)

        # Move to relative velocity frame
        linear_velocity = desired_twist.get_linear_velocity()
        angular_velocity = desired_twist.get_angular_velocity()

        # initialize controller : Fcomannd = I*x_dd + D*x_d + K*x
        ctrl_dissipative = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE)
        ctrl_dissipative.set_parameter_value(
            "damping", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )

        ctrl_dissipative.set_parameter_value(
            "stiffness", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )

        ctrl_dissipative.set_parameter_value(
            "inertia", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )

        # Approach position - Damping linear velocity
        D = np.zeros((6, 6))
        D[:3, :3] = np.diag(
            [
                self.linear_principle_damping,
                self.linear_orthogonal_damping,
                self.linear_orthogonal_damping,
            ]
        )

        E = get_orthogonal_basis(linear_velocity)
        if not np.allclose(E @ E.T, np.eye(3)):
            # Sanity check [remove in the future]
            # Error -> non-orthogonal basis!!!
            breakpoint()

        D[:3, :3] = E @ D[:3, :3] @ E.T
        ctrl_dissipative.set_parameter_value("damping", D, sr.ParameterType.MATRIX)

        des_state = sr.CartesianState(
            state.ee_state.get_name(),
            state.ee_state.get_reference_frame(),
        )

        des_state.set_linear_velocity(linear_velocity)
        des_state.set_angular_velocity(angular_velocity)

        tmp_state = copy.deepcopy(state.ee_state)
        tmp_state.set_force(0, 0, 0)
        tmp_state.set_torque(0, 0, 0)

        return ctrl_dissipative.compute_command(
            desired_twist, tmp_state, state.jacobian
        )

    def controller_velocity_impedance_simplified(self, state, desired_twist=None):
        # This is actually a psotion disspipative
        if desired_twist is None:
            desired_twist = sr.CartesianTwist(
                self.ds_dissipative.evaluate(state.ee_state)
            )
            desired_twist.clamp(self.clamp_linear, self.clamp_angular * 2.0)

        ctrl_impedance = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE)
        ctrl_impedance.set_parameter_value(
            "stiffness", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )
        # ctrl_impedance.set_parameter_value(
        #     "damping", np.zeros((6, 6)), sr.ParameterType.MATRIX)
        ctrl_impedance.set_parameter_value(
            "inertia", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )

        # Approach position - Damping linear velocity
        D = np.zeros((6, 6))
        D[3:, 3:] = np.eye(3) * self.angular_damping
        ctrl_impedance.set_parameter_value("damping", D, sr.ParameterType.MATRIX)

        tmp_state = copy.deepcopy(state.ee_state)
        tmp_state.set_force(np.zeros(3))
        tmp_state.set_torque(np.zeros(3))
        tmp_state.set_orientation([1, 0, 0, 0])

        torque = D[3:, 3:] @ (
            desired_twist.get_angular_velocity() - tmp_state.get_angular_velocity()
        )
        print("\n")
        print("AngVel: impedance", desired_twist.get_angular_velocity())
        print("Torque simplified- impedance", torque)

        ########## Comparison ##########
        desired_virutal_twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        desired_virutal_twist.clamp(self.clamp_linear, self.clamp_angular)

        des_virt_state = sr.CartesianState(
            state.ee_state.get_name(),
            state.ee_state.get_reference_frame(),
        )
        des_virt_state.set_angular_velocity(
            desired_virutal_twist.get_angular_velocity()
        )

        angular_velocity = des_virt_state.get_angular_velocity()
        if angular_norm := np.linalg.norm(angular_velocity):
            dt = 1
            theta = angular_norm * dt * 0.5
            q_w = math.cos(theta)
            q_vec = angular_velocity / angular_norm * math.sin(theta)

            # tmp_state.set_orientation(np.hstack((q_w, q_vec)))
            des_virt_state.set_orientation(np.hstack((q_w, q_vec)))

        des_virt_rot = Rotation(np.append(q_vec, q_w))
        qq = tmp_state.get_orientation()
        tmp_rot = Rotation([qq[1], qq[2], qq[3], qq[0]])

        delta_rot = des_virt_rot * tmp_rot.inv()
        # delta_rot = tmp_rot * des_virt_rot.inv()
        delta_rotvec = delta_rot.as_rotvec()

        delta_state = des_virt_state - desired_virutal_twist

        return ctrl_impedance.compute_command(desired_twist, tmp_state, state.jacobian)

    def controller_velocity_impedance_with_integration(self, state, desired_twist=None):
        ### Velocity IMPEDANCE (ANGULAR) ###
        # Damp / reduce velocity [to zero...]
        if desired_twist is None:
            desired_twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
            desired_twist.clamp(self.clamp_linear, self.clamp_angular)

        ctrl_impedance = create_cartesian_controller(CONTROLLER_TYPE.IMPEDANCE)
        K = np.zeros((6, 6))
        K[3:, 3:] = np.eye(3) * self.angular_stiffness
        ctrl_impedance.set_parameter_value("stiffness", K, sr.ParameterType.MATRIX)

        D = np.zeros((6, 6))
        D[3:, 3:] = np.eye(3) * self.angular_damping
        ctrl_impedance.set_parameter_value("damping", D, sr.ParameterType.MATRIX)

        ctrl_impedance.set_parameter_value(
            "inertia", np.zeros((6, 6)), sr.ParameterType.MATRIX
        )

        tmp_state = copy.deepcopy(state.ee_state)
        tmp_state.set_force(np.zeros(3))
        tmp_state.set_torque(np.zeros(3))
        tmp_state.set_orientation([1.0, 0.0, 0.0, 0.0])

        des_virt_state = sr.CartesianState(
            state.ee_state.get_name(),
            state.ee_state.get_reference_frame(),
        )
        des_virt_state.set_angular_velocity(desired_twist.get_angular_velocity())

        angular_velocity = des_virt_state.get_angular_velocity()
        if angular_norm := np.linalg.norm(angular_velocity):
            dt = 1
            theta = angular_norm * dt * 0.5
            q_w = math.cos(theta)
            q_vec = angular_velocity / angular_norm * math.sin(theta)

            des_virt_state.set_orientation(np.hstack((q_w, q_vec)))

        pos_error = des_virt_state.get_angular_velocity() * 0.5
        torque = K[3:, 3:] @ pos_error + D[3:, 3:] @ (
            des_virt_state.get_angular_velocity() - tmp_state.get_angular_velocity()
        )
        proportional = K[3:, 3:] @ des_virt_state.get_angular_velocity() * 0.5
        damping = D[3:, 3:] @ (
            des_virt_state.get_angular_velocity() - tmp_state.get_angular_velocity()
        )
        torque = proportional + damping
        print("\n\n")
        print("Prop (imp):", proportional)
        print("Error Prop", pos_error)
        print("AngVel: impedance", des_virt_state.get_angular_velocity())
        print("Torque: impedance", torque)

        return ctrl_impedance.compute_command(des_virt_state, tmp_state, state.jacobian)

    def controller_callback(self) -> None:
        command = CommandMessage()
        command.control_type = [ControlType.EFFORT.value]

        state = self.robot.get_state()
        if not state:
            print("Waiting for a state.")
            self.rate.sleep()
            return

        # Current force / torque are eliminated in order to not 'overcompensate' them
        state.ee_state.set_force(np.zeros(3))
        state.ee_state.set_torque(np.zeros(3))

        desired_twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        desired_twist.clamp(self.clamp_linear, self.clamp_angular)

        dissipative_twist = sr.CartesianTwist(
            self.ds_dissipative.evaluate(state.ee_state)
        )
        dissipative_twist.clamp(self.clamp_linear, self.clamp_angular * 2.0)

        cmnd_compliant_twist = self.controller_compliant_twist(state)
        # cmnd_impedance = self.controller_velocity_impedance_with_integration(state, desired_twist)
        cmnd_dissipative = self.controller_dissipative(state, dissipative_twist)

        # Compare the various Controllers
        print("T | Compliant", np.round(cmnd_compliant_twist.get_torques(), 3))
        # print("T | Dissipat.", np.round(cmnd_dissipative.get_torques(), 3))
        # print("T | Impedance (with Integration)", np.round(cmnd_impedance.get_torques(), 3))
        print("T | Dissipative", np.round(cmnd_dissipative.get_torques(), 3))

        command_torques = sr.JointTorques(cmnd_dissipative)
        # command_torques = sr.JointTorques(cmnd_compliant_twist)

        command.joint_state = state.joint_state
        command.joint_state.set_torques(command_torques.get_torques())
        self.robot.send_command(command)

        # print("Done --- I'm out for a bit....")
        self.rate.sleep()
        # print("Had the nap.")


if __name__ == "__main__":
    print("Starting Joint Space....")
    rclpy.init()
    # rospy.init_node("test", anonymous=True)
    robot_interface = RobotInterface("*:1601", "*:1602")

    # Spin in a separate thread
    controller = CartesianSpaceController(
        robot=robot_interface, freq=100, is_simulation=False
    )

    # thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    # thread.start()

    try:
        # controller.run()
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    controller.destroy_node()

    rclpy.shutdown()
    # thread.join()
