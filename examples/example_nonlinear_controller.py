#!/usr/bin/env python3
from typing import Optional
import math

import numpy as np
from scipy.spatial.transform import Rotation

# ROS related
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# LASA Libraries
import state_representation as sr
from controllers import create_cartesian_controller, CONTROLLER_TYPE
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage

# Custom libraries
from vartools.states import ObjectPose
from roam.dynamics import DynamicDynamics, SimpleCircularDynamics
from roam.multi_obstacle_avoider import MultiObstacleAvoider
from roam.dynamics.projected_rotation_dynamics import (
    ProjectedRotationDynamics,
)

# Local library
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface
from franka_avoidance.human_optitrack_container import create_optitrack_human


class NonlinearAvoidanceController(Node):
    dimension = 3

    def __init__(
        self,
        robot,
        freq: float = 100,
        node_name="nonlinear_avoidance_controller",
        is_simulation: bool = False,
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
                "linear_principle_damping", 20.0, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "linear_orthogonal_damping", 20.0, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "angular_stiffness", 1.0, sr.ParameterType.DOUBLE
            )
            self.ctrl.set_parameter_value(
                "angular_damping", 1.0, sr.ParameterType.DOUBLE
            )

        # Geut robot state to set up the target in the same frame
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
            "gain", [50.0, 50.0, 50.0, 10.0, 10.0, 10.0], sr.ParameterType.DOUBLE_ARRAY
        )
        self.ds.set_parameter_value(
            "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
        )

        # Create circular - circular dynamics
        pose = ObjectPose(
            position=np.array([1.0, 0, 1]),
            # orientation=Rotation.from_euler("y", math.pi / 2),
        )
        main_ds = SimpleCircularDynamics(pose=pose, radius=0.3)
        ds_of_base = SimpleCircularDynamics(radius=1, pose=pose)
        self.dynamic_dynamics = DynamicDynamics(
            main_dynamics=main_ds, dynamics_of_base=ds_of_base
        )

        self.rotation_projector = ProjectedRotationDynamics(
            attractor_position=self.dynamic_dynamics.position,
            initial_dynamics=self.dynamic_dynamics,
            reference_velocity=lambda x: x - self.dynamic_dynamics.position,
        )

        self.human_with_limbs = create_optitrack_human()
        self.avoider = MultiObstacleAvoider(
            obstacle=self.human_with_limbs,
            initial_dynamics=self.dynamic_dynamics,
            convergence_dynamics=self.rotation_projector,
        )

        self.command = CommandMessage()
        self.command.control_type = [ControlType.EFFORT.value]

        self.timer = self.create_timer(period, self.controller_callback)
        print("Finish init.")

    def publisher_trajectory_initial(
        self, start_position: np.ndarray, it_max: int = 100, delta_time: float = 0.1
    ) -> None:
        positions = np.zeros((self.dimension, it_max + 1))
        for ii in range(it_max):
            position = positions[:, ii]

            velocity = self.dynamic_dynamics.evaluate(position)
            root_obstacle = self.human_with_limbs.get_component(
                self.human_with_limbs.root_id
            )

            linear_velocity = (
                self.rotation_projector.evaluate_convergence_around_obstacle(
                    position, root_obstacle
                )
            )

            # final_velocity = self.avoider.evaluate(position)

            position[:, ii + 1] = position[:, ii + 1] + velocity * delta_time

    def controller_callback(self) -> None:
        state = self.robot.get_state()

        self.human_with_limbs.update()

        if not state:
            return

        twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        twist.clamp(0.25, 0.2)

        # Compute Avoidance-DS
        position = state.ee_state.get_position()
        avoidance_velocity = self.avoider.evaluate(position)
        # twist.set_linear_velocity(avoidance_velocity)

        # Compute the torques
        cartesian_command = self.ctrl.compute_command(
            twist, state.ee_state, state.jacobian
        )

        self.command.joint_state = state.joint_state
        # self.command_torques = sr.JointTorques(cartesian_command)
        # self.command.joint_state.set_torques(self.command_torques.get_torques())
        self.command.joint_state.set_torques(cartesian_command.get_torques())

        self.robot.send_command(self.command)


if __name__ == "__main__":
    print("Starting CartesianTwist controller ...")
    rclpy.init()
    robot_interface = RobotInterface("*:1601", "*:1602")

    controller = NonlinearAvoidanceController(
        robot=robot_interface, freq=50, is_simulation=False
    )

    try:
        print("Startinit.")
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
