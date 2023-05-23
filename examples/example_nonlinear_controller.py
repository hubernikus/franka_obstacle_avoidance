#!/usr/bin/env python3
from typing import Optional
import math
import copy
import time

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
from vartools.states import Pose

from vartools.directional_space import get_directional_weighted_sum

from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid

from nonlinear_avoidance.dynamics import DynamicDynamics, SimpleCircularDynamics
from nonlinear_avoidance.multi_obstacle import MultiObstacle
from nonlinear_avoidance.multi_obstacle_avoider import MultiObstacleAvoider
from nonlinear_avoidance.multi_obstacle_container import MultiObstacleContainer
from nonlinear_avoidance.dynamics.projected_rotation_dynamics import (
    ProjectedRotationDynamics,
)

# Local library
# from franka_avoidance.human_optitrack_container import create_optitrack_human
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface
from franka_avoidance.franka_joint_space import FrankaJointSpace
from franka_avoidance.velocity_publisher import VelocityPublisher


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


def create_target_ds(target):
    ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
    ds.set_parameter_value(
        "gain", [50.0, 50.0, 50.0, 5.0, 5.0, 5.0], sr.ParameterType.DOUBLE_ARRAY
    )
    ds.set_parameter_value(
        "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
    )
    return ds


# self.ctrl.set_parameter_value(
#                "linear_principle_damping", 1.0, sr.ParameterType.DOUBLE
#            )
#            self.ctrl.set_parameter_value(
#                "linear_orthogonal_damping", 1.0, sr.ParameterType.DOUBLE
#            )
#            self.ctrl.set_parameter_value(
#                "angular_stiffness", 0.5, sr.ParameterType.DOUBLE
#            )
#            self.ctrl.set_parameter_value(
#                "angular_damping", 0.5, sr.ParameterType.DOUBLE
#            )
#        else:
#            self.ctrl.set_parameter_value(
#                "linear_principle_damping", 50.0, sr.ParameterType.DOUBLE
#            )
#            self.ctrl.set_parameter_value(
#                "linear_orthogonal_damping", 50.0, sr.ParameterType.DOUBLE
#            )
#            self.ctrl.set_parameter_value(
#                "angular_stiffness", 2.0, sr.ParameterType.DOUBLE
#            )
#            self.ctrl.set_parameter_value(
#                "angular_damping", 2.0, sr.ParameterType.DOUBLE
#            )


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

        self.max_velocity = 0.25
        self.joint_robot = FrankaJointSpace()

        if is_simulation:
            self.ctrl = create_simulation_compliant_controller()
        else:
            self.ctrl = create_realworld_compliant_controller()

        # Get robot state to set up the target in the same frame
        while not (state := self.robot.get_state()) and rclpy.ok():
            print("Awaiting first robot-state.")

        if target is None:
            quat = np.array([0.0, 1.0, 0.0, 1.0])
            quat = quat / np.linalg.norm(quat)
            # quat = np.array([0.0, 1.0, 0.0, 0.0])
            target = sr.CartesianPose(
                state.ee_state.get_name(),
                np.array([0.3, 0.1, 0.8]),
                quat,
                state.ee_state.get_reference_frame(),
            )

        self.ds = create_target_ds(target)
        self.create_circular_dynamics()
        self.create_obstacle_environment(margin_absolut = 0.2)
        # self.human_with_limbs = create_flexilimb_human()
        
        robot_frame = "panda_link0"
        self.inital_velocity_publisher = VelocityPublisher("initial", robot_frame)
        self.rotated_velocity_publisher = VelocityPublisher("rotated", robot_frame)

        self.command = CommandMessage()
        # self.command.control_type = [ControlType.EFFORT.value]
        self.command.control_type = [ControlType.VELOCITY.value]

        self.timer = self.create_timer(period, self.controller_callback)
        print("Finish init.")

    def create_circular_dynamics(self):
        # Create circular - circular dynamics
        self.center_pose = Pose(
            np.array([0.9, -0.2, 0.8]),
            orientation=Rotation.from_euler("y", -math.pi / 2),
        )
        pose_base = copy.deepcopy(self.center_pose)
        pose_base.position = pose_base.position
        self.ds_of_base = SimpleCircularDynamics(pose=pose_base, radius=0.2)
        # self.main_ds = SimpleCircularDynamics(pose=pose, radius=0.5)

        # self.dynamic_dynamics = DynamicDynamics(
        #     main_dynamics=self.main_ds,
        #     dynamics_of_base=self.ds_of_base,
        #     frequency=freq,
        # )
        # self.dynamic_dynamics.time_step_of_base_movement = 1.0 / 10  # Try fast !

        # self.rotation_projector = ProjectedRotationDynamics(
        #     attractor_position=self.dynamic_dynamics.position,
        #     initial_dynamics=self.dynamic_dynamics,
        #     reference_velocity=lambda x: x - self.dynamic_dynamics.position,
        # )

    def create_obstacle_environment(self, margin_absolut: float) -> None:
        obstacle1 = MultiObstacle(self.center_pose)
        obstacle1.set_root(
            Cuboid(
                axes_length=np.array([0.5, 0.5, 0.3]),
                pose=Pose(
                    np.zeros(self.dimension), orientation=Rotation.from_euler("x", 0.0)
                ),
                margin_absolut=margin_absolut,
            ),
        )

        self.rotation_projector = ProjectedRotationDynamics(
            attractor_position=self.ds_of_base.pose.position,
            initial_dynamics=self.ds_of_base,
            reference_velocity=lambda x: x - self.ds_of_base.pose.position,
        )

        self.container = MultiObstacleContainer()
        self.container.append(obstacle1)

        self.avoider = MultiObstacleAvoider(
            obstacle_container=self.container,
            initial_dynamics=self.ds_of_base,
            convergence_dynamics=self.rotation_projector,
        )

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

            final_velocity = (
                self.rotation_projector.evaluate_convergence_around_obstacle(
                    position, root_obstacle
                )
            )

            # final_velocity = self.avoider.evaluate(position)
            position[:, ii + 1] = position[:, ii + 1] + velocity * delta_time

    def controller_callback(self) -> None:
        state = self.robot.get_state()
        # self.human_with_limbs.update()

        if not state:
            return

        twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        twist.clamp(self.max_velocity, 0.5)

        # Compute Avoidance-DS
        position = state.ee_state.get_position()

        desired_velocity = self.ds_of_base.evaluate(position)
        self.inital_velocity_publisher.publish(position, desired_velocity)

        tic = time.perf_counter()
        # desired_velocity = self.avoider.evaluate(position)
        desired_velocity = self.ds_of_base.evaluate(position)
        toc = time.perf_counter()
        print(f"Timer took: {toc - tic:0.4f} s")
        self.rotated_velocity_publisher.publish(position, desired_velocity)

        # One time-step of the base-system.
        if np.linalg.norm(desired_velocity) > self.max_velocity:
            desired_velocity = (
                desired_velocity / np.linalg.norm(desired_velocity) * self.max_velocity
            )

        twist.set_linear_velocity(desired_velocity)

        self.command.joint_state = state.joint_state
        # desired_joint_vel = np.linalg.lstsq(
        #     state.jacobian.data(), twist.get_twist(), rcond=None
        # )[0]

        # desired_joint_vel = desired_joint_vel * 0.7  # Slow it down for testing

        # final_joint_velocity = self.joint_robot.get_limit_avoidance_velocity(
        #     joint_position=state.joint_state.get_positions(),
        #     joint_velocity=desired_joint_vel,
        #     jacobian=state.jacobian.data(),
        # )
        final_joint_velocity = (
            self.joint_robot.get_limit_avoidance_velocity_from_cartesian(
                twist=twist,
                state=state,
            )
        )

        if np.any(np.isnan(final_joint_velocity)):
            breakpoint()  # For debugging

        self.command.joint_state.set_velocities(final_joint_velocity)
        self.robot.send_command(self.command)


if __name__ == "__main__":
    print("Starting CartesianTwist controller ...")
    rclpy.init()
    # robot_interface = RobotInterface("*:1601", "*:1602")
    # robot_id = 16
    robot_id = 17
    robot_interface = RobotInterface(f"*:{robot_id}01", f"*:{robot_id}02")

    controller = NonlinearAvoidanceController(
        robot=robot_interface, freq=200, is_simulation=False
    )

    try:
        print("Startinit.")
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
