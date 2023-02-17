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
from vartools.directional_space import get_directional_weighted_sum

from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid
from roam.dynamics import DynamicDynamics, SimpleCircularDynamics
from roam.multi_obstacle_avoider import MultiObstacleAvoider
from roam.dynamics.projected_rotation_dynamics import (
    ProjectedRotationDynamics,
)

# Local library
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface
from franka_avoidance.human_optitrack_container import create_optitrack_human


class FrankaJointSpace:
    q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    q_delta = q_max - q_min
    q_mean = 0.5 * (q_min + q_max)

    boundary = Cuboid(center_position=q_mean, axes_length=q_delta, is_boundary=True)
    boundary.boundary_power_factor = 2.0

    # def __init__(self, weight_power: float = 2.0) -> None:
    #     self.weight_power =

    def __init__(self, weight_power: float = 2):
        self.weight_power = weight_power

    def get_gamma_weight(
        self,
        relative_position: np.ndarray,
        gamma_max: float = 1.5,
        gamma_min: float = 1.0,
    ) -> float:
        gamma = self.boundary.get_gamma(relative_position, in_obstacle_frame=True)
        print("rel pos", relative_position)
        print("gamma", gamma)
        if gamma > gamma_max:
            return 0
        elif gamma < gamma_min:
            return 1

        return (gamma_max - gamma) / (gamma_max - gamma_min)

    def get_limit_avoidance_velocity(
        self,
        joint_position: np.ndarray,
        joint_velocity: np.ndarray,
        jacobian: np.ndarray,
    ) -> np.ndarray:
        # TODO: watch out - switching at the back -> use RotationalAvoider (!)
        # IDEAS:
        # - Move towards null-space direction (?) / use (inverted) normal pulling ?
        # - Discontuinity when moving directly towards attractor -> induce a decreasing weight (!)
        # ==> See paper...
        if not (joint_speed := np.linalg.norm(joint_velocity)):
            return joint_velocity

        joint_velocity = joint_velocity / joint_speed

        relative_position = joint_position - self.q_mean

        danger_weight = self.get_gamma_weight(relative_position)
        if danger_weight <= 0:
            return joint_velocity

        print("Danger weight", danger_weight)

        no_move_direction = self.get_no_move_direction(jacobian)
        normal = self.boundary.get_normal_direction(
            relative_position, in_obstacle_frame=True
        )
        no_move_weight = np.dot(no_move_direction, normal)
        if no_move_weight < 0:
            no_move_direction = -no_move_direction
            no_move_weight = -no_move_weight

        elif np.isclose(no_move_weight, 0):
            rotated_velocity = get_directional_weighted_sum(
                null_direction=normal,
                weights=np.array([(1 - danger_weight)]),
                directions=np.array([joint_velocity]).reshape(-1, 1),
            )

            return rotated_velocity * joint_speed

        rotated_velocity = get_directional_weighted_sum(
            null_direction=normal,
            weights=np.array([(1 - danger_weight), danger_weight * no_move_weight]),
            directions=np.vstack((joint_velocity, no_move_direction)).T,
        )
        # Keep velocity magnitude
        return rotated_velocity * joint_speed

    @staticmethod
    def get_no_move_direction(jacobian: np.ndarray):
        _, _, uv = np.linalg.svd(jacobian, compute_uv=True, full_matrices=True)
        return uv[-1, :]

    @classmethod
    def get_velocity_away_from_limits(
        cls, joint_position: np.ndarray, jacobian: np.ndarray
    ) -> np.ndarray:
        # PROBLEM: this can lead to jittering when in-between two cornerns...
        dist_qmin = cls.q_min - joint_position
        dist_qmax = cls.q_max - joint_position

        dist_q = np.vstack((dist_qmin, dist_qmax)) / (
            np.tile(cls.q_delta * 0.5, (2, 1))
        )

        idx_closest = np.argmin(np.abs(dist_q))
        idx_closest = np.unravel_index(idx_closest, dist_q.shape)

        weight = 1 - abs(dist_q[idx_closest])

        weight = weight**cls.weight_power

        if not weight:
            return np.zeros_like(joint_position)

        null_velocity = cls.find_null_space_velocity(jacobian)

        if (null_velocity[idx_closest[1]] > 0 and idx_closest[0] == 1) or (
            null_velocity[idx_closest[1]] < 0 and idx_closest[0] == 0
        ):
            null_velocity = -null_velocity
            # weight = -weight

        return null_velocity, weight


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

        # Get robot state to set up the target in the same frame
        while not (state := self.robot.get_state()) and rclpy.ok():
            print("Awaiting first state.")

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

        self.ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds.set_parameter_value(
            "gain", [50.0, 50.0, 50.0, 5.0, 5.0, 5.0], sr.ParameterType.DOUBLE_ARRAY
        )
        self.ds.set_parameter_value(
            "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
        )

        # Create circular - circular dynamics
        pose = ObjectPose(
            np.array([0.3, 0.0, 0.8]),
            orientation=Rotation.from_euler("y", -math.pi / 2),
        )
        self.ds_of_base = SimpleCircularDynamics(radius=0.1, pose=pose)
        self.main_ds = SimpleCircularDynamics(pose=pose, radius=0.3)

        # self.dynamic_dynamics = DynamicDynamics(
        #     main_dynamics=self.main_ds, dynamics_of_base=self.ds_of_base
        # )

        # self.dynamic_dynamics = DynamicDynamics(
        #     main_dynamics=main_ds, dynamics_of_base=ds_of_base
        # )

        # self.rotation_projector = ProjectedRotationDynamics(
        #     attractor_position=self.dynamic_dynamics.position,
        #     initial_dynamics=self.dynamic_dynamics,
        #     reference_velocity=lambda x: x - self.dynamic_dynamics.position,
        # )

        self.rotation_projector = ProjectedRotationDynamics(
            attractor_position=self.ds_of_base.pose.position,
            initial_dynamics=self.ds_of_base,
            reference_velocity=lambda x: x - self.ds_of_base.pose.position,
        )

        self.human_with_limbs = create_optitrack_human()
        self.avoider = MultiObstacleAvoider(
            obstacle=self.human_with_limbs,
            initial_dynamics=self.ds_of_base,
            convergence_dynamics=self.rotation_projector,
        )

        self.command = CommandMessage()
        # self.command.control_type = [ControlType.EFFORT.value]
        self.command.control_type = [ControlType.VELOCITY.value]

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

            final_velocity = (
                self.rotation_projector.evaluate_convergence_around_obstacle(
                    position, root_obstacle
                )
            )

            # final_velocity = self.avoider.evaluate(position)

            position[:, ii + 1] = position[:, ii + 1] + velocity * delta_time

    def controller_callback(self) -> None:
        state = self.robot.get_state()
        # print(state)
        self.human_with_limbs.update()

        if not state:
            return

        twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        twist.clamp(self.max_velocity, 0.5)
        # print(twist)

        # Compute Avoidance-DS
        position = state.ee_state.get_position()
        # avoidance_velocity = self.avoider.evaluate(position)
        desired_velocity = self.main_ds.evaluate(position)
        # twist.set_linear_velocity(avoidance_velocity)

        if np.linalg.norm(desired_velocity) > self.max_velocity:
            desired_velocity = (
                desired_velocity / np.linalg.norm(desired_velocity) * self.max_velocity
            )

        twist.set_linear_velocity(desired_velocity)

        # Compute the torques
        cartesian_command = self.ctrl.compute_command(
            twist, state.ee_state, state.jacobian
        )

        self.command.joint_state = state.joint_state
        # self.command_torques = sr.JointTorques(cartesian_command)
        # self.command.joint_state.set_torques(self.command_torques.get_torques())
        # print("torques", cartesian_command.get_torques())
        # self.command.joint_state.set_torques(cartesian_command.get_torques())
        # self.robot.send_command(self.command)

        # (
        #     nullspace_vel,
        #     null_weight,
        # ) = FrankaJointSpace.get_velocity_away_from_limits(
        #     state.joint_state.get_positions(), state.jacobian.data()
        # )

        # if not np.isclose(np.linalg.norm(state.jacobian.data() @ nullspace_vel), 0):
        #     # Sanity check
        #     breakpoint()

        desired_joint_vel = np.linalg.lstsq(
            state.jacobian.data(), twist.get_twist(), rcond=None
        )[0]

        desired_joint_vel = desired_joint_vel * 0.3  # Slow it down for testing

        final_joint_velocity = self.joint_robot.get_limit_avoidance_velocity(
            joint_position=state.joint_state.get_positions(),
            joint_velocity=desired_joint_vel,
            jacobian=state.jacobian.data(),
        )

        self.command.joint_state.set_velocities(final_joint_velocity)

        self.robot.send_command(self.command)


if __name__ == "__main__":
    print("Starting CartesianTwist controller ...")
    rclpy.init()
    robot_interface = RobotInterface("*:1601", "*:1602")

    controller = NonlinearAvoidanceController(
        robot=robot_interface, freq=20, is_simulation=False
    )

    try:
        print("Startinit.")
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
