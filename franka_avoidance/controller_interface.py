#!/usr/bin/env python3
import copy

import numpy as np


# LASA Libraries
import state_representation as sr
from controllers import create_cartesian_controller, CONTROLLER_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage


def create_simulation_compliant_controller():
    ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
    ctrl.set_parameter_value("linear_principle_damping", 1.0, sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("linear_orthogonal_damping", 1.0, sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("angular_stiffness", 0.5, sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("angular_damping", 0.5, sr.ParameterType.DOUBLE)
    return ctrl


def create_realworld_compliant_controller():
    ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
    ctrl.set_parameter_value("linear_principle_damping", 50.0, sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("linear_orthogonal_damping", 50.0, sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("angular_stiffness", 2.0, sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("angular_damping", 3.0, sr.ParameterType.DOUBLE)
    return ctrl


class VelocityCommandHandler:
    def __init__(self) -> None:
        self.command = CommandMessage()
        self.command.control_type = [ControlType.VELOCITY.value]

        self.max_velocity = 0.1

    # def update_state(self, state) -> None:
    #     self.command.joint_state = copy.deepcopy(state.joint_state)
    #     self.command.joint_state.set_torques(np.zeros(self.robot.dof))

    def update_from_cartesian_twist(self, twist, state) -> None:
        self.command.joint_state = copy.deepcopy(state.joint_state)
        self.command.joint_state.set_zero()

        joint_velocity = np.linalg.lstsq(
            state.jacobian.data(), twist.get_twist(), rcond=None
        )[0]
        self.command.joint_state.set_velocities(joint_velocity)

        if self.max_velocity is not None:
            self.limit_joint_velocity(self.max_velocity)

    def limit_joint_velocity(self, max_velocity: float) -> None:
        joint_velocity = self.command.joint_state.get_velocities()
        joint_velocity = np.clip(joint_velocity, -max_velocity, max_velocity)
        self.command.joint_state.set_velocities(joint_velocity)

    def get_command(self):
        joint_velocity = self.command.joint_state.get_velocities()
        if np.any(np.isnan(joint_velocity)):
            breakpoint()  # For debugging

        if joint_velocity.shape[0] != 7:
            breakpoint()  # For debugging

        return self.command


class ImpedanceCommandHandler:
    def __init__(self, ctrl) -> None:
        self.command = CommandMessage()
        self.command.control_type = [ControlType.EFFORT.value]

        self.ctrl = ctrl

    @classmethod
    def create_realworld(cls):
        ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
        ctrl.set_parameter_value(
            "linear_principle_damping", 50.0, sr.ParameterType.DOUBLE
        )
        ctrl.set_parameter_value(
            "linear_orthogonal_damping", 50.0, sr.ParameterType.DOUBLE
        )
        ctrl.set_parameter_value("angular_stiffness", 2.0, sr.ParameterType.DOUBLE)
        ctrl.set_parameter_value("angular_damping", 3.0, sr.ParameterType.DOUBLE)
        # ctrl = create_realworld_compliant_controller()
        return cls(ctrl)

    @classmethod
    def create_simulation(cls):
        ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
        ctrl.set_parameter_value(
            "linear_principle_damping", 1.0, sr.ParameterType.DOUBLE
        )
        ctrl.set_parameter_value(
            "linear_orthogonal_damping", 1.0, sr.ParameterType.DOUBLE
        )
        ctrl.set_parameter_value("angular_stiffness", 0.5, sr.ParameterType.DOUBLE)
        ctrl.set_parameter_value("angular_damping", 0.5, sr.ParameterType.DOUBLE)
        # ctrl = create_simulation_compliant_controller()
        return cls(ctrl)

    def update_from_cartesian_twist(self, twist, state):
        cartesian_command = self.ctrl.compute_command(
            twist, state.ee_state, state.jacobian
        )
        self.command.joint_state = copy.deepcopy(state.joint_state)
        self.command.joint_state.set_zero()
        self.command.joint_state.set_torques(cartesian_command.get_torques())

    def get_command(self):
        return self.command
