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

# LASA Libraries
import state_representation as sr
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE

# Custom libraries
from vartools.states import Pose


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
from franka_avoidance.trajectory_publisher import TrajectoryPublisher
from franka_avoidance.controller_interface import (
    VelocityCommandHandler,
    ImpedanceCommandHandler,
)


class LineFollowingOscillation:
    # Currently this is only for ideal (path) following
    # No continuity is considered
    def __init__(self, range=[0.3, 0.4]):
        self.range = range

        self._direction = 1
        self.step = 0.01

    def evaluate(self, position):
        if self._direction > 0:
            position[0] += self.step
            if position[1] > range[1]:
                self._direction = -1

        elif self._direction < 0:
            position[0] -= self.step
            if position[0] < range[0]:
                self._direction = 2

        else:
            raise ValueError("Direction is not set.")

        return position


def create_target_ds(target: sr.CartesianPose):
    ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
    ds.set_parameter_value(
        "gain", [50.0, 50.0, 50.0, 5.0, 5.0, 5.0], sr.ParameterType.DOUBLE_ARRAY
    )
    ds.set_parameter_value(
        "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
    )
    return ds


def create_cartesian_pose(
    state, position: np.ndarray, orientation: Optional[np.ndarray] = None
):
    if orientation is None:
        # Default pose is the robot pointing down
        orientation = np.array([0.0, 1.0, 0.0, 0.0])
    else:
        # make sure it's normalized
        orientation = orientation / np.linalg.norm(orientation)

    new_pose = sr.CartesianPose(
        state.ee_state.get_name(),
        position,
        orientation,
        state.ee_state.get_reference_frame(),
    )

    return new_pose


class NonlinearAvoidanceController(Node):
    dimension = 3

    def __init__(
        self,
        robot,
        freq: float = 100,
        node_name="nonlinear_avoidance_controller",
        is_simulation: bool = False,
        # is_velocity_controller: bool = True,
        is_velocity_controller: bool = False,
        target: Optional[sr.CartesianPose] = None,
    ):
        super().__init__(node_name)
        self.robot = robot
        self.rate = self.create_rate(freq)
        period = 1.0 / freq

        self.max_velocity = 0.25
        self.joint_robot = FrankaJointSpace()

        if is_velocity_controller:
            self.command_handler = VelocityCommandHandler()
        # Otherwise -> CartesianTwist
        elif is_simulation:
            self.command_handler = ImpedanceCommandHandler.create_simulation()
        else:
            print("Doing real-world impedance")
            self.command_handler = ImpedanceCommandHandler.create_realworld()

        # if True:
        #    breakpoint()

        # Get robot state to set up the target in the same frame
        while not (state := self.robot.get_state()) and rclpy.ok():
            # breakpoint()
            print("Awaiting first robot-state.")

        if target is None:
            target = create_cartesian_pose(
                state,
                position=np.array([0.3, 0.1, 0.8]),
                orientation=None,
                # orientation=np.array([0.0, 1.0, 0.0, 0.0]),
            )

        self.ds = create_target_ds(target)
        # self.create_circular_dynamics()
        self.create_circular_conveyer_dynamics()
        self.create_obstacle_environment(margin_absolut=0.2)
        # self.human_with_limbs = create_flexilimb_human()

        robot_frame = "panda_link0"
        self.inital_velocity_publisher = VelocityPublisher("initial", robot_frame)
        self.rotated_velocity_publisher = VelocityPublisher("rotated", robot_frame)
        self.initial_trajectory = TrajectoryPublisher(
            self.dynamics.evaluate, "initial", robot_frame
        )
        # self.avoider_trajectory = TrajectoryPublisher(
        #     self.avoider.evaluate, "initial", robot_frame
        # )

        self.timer = self.create_timer(period, self.controller_callback)
        print("Finish initialization.")

    def create_circular_conveyer_dynamics(self):
        self.center_pose = Pose(
            np.array([0.5, 0.0, 0.2]),
            orientation=Rotation.from_euler("x", 0.0),
        )
        self.dynamics = SimpleCircularDynamics(pose=self.center_pose, radius=0.1)

        self.it_center = 0

    def update_center_linear(self, period: int = 1000) -> None:
        center1 = np.array([0.4, 0.0, 0.2])
        center2 = np.array([0.8, 0.0, 0.2])

        self.it_center += 1

        progress = self.it_center % period
        progress = abs(progress - period / 2) / period

        center = center1 * (1 - progress) + center2 * progress
        self.dynamics.pose.position = center

        # TODO: update additional settings / centers if necessary (!)

    def create_circular_dynamics(self):
        # Create circular - circular dynamics
        self.center_pose = Pose(
            np.array([0.9, -0.2, 0.8]),
            orientation=Rotation.from_euler("y", -math.pi / 2),
        )

        pose_base = copy.deepcopy(self.center_pose)
        pose_base.position = pose_base.position
        self.dynamics = SimpleCircularDynamics(pose=pose_base, radius=0.2)
        # self.main_ds = SimpleCircularDynamics(pose=pose, radius=0.5)

        # self.dynamic_dynamics = DynamicDynamics(
        #     main_dynamics=self.main_ds,
        #     dynamics_of_base=self.dynamics
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
            attractor_position=self.dynamics.pose.position,
            initial_dynamics=self.dynamics,
            reference_velocity=lambda x: x - self.dynamics.pose.position,
        )

        self.container = MultiObstacleContainer()
        self.container.append(obstacle1)

        self.avoider = MultiObstacleAvoider(
            obstacle_container=self.container,
            initial_dynamics=self.dynamics,
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

    def create_velocity_command(self, state) -> CommandMessage:
        command = CommandMessage()
        #  command.control_type = [ControlType.EFFORT.value]
        command.control_type = [ControlType.VELOCITY.value]

        command.joint_state = copy.deepcopy(state.joint_state)
        command.joint_state.set_torques(np.zeros(self.robot.dof))

        return command

    def controller_callback(self) -> None:
        state = self.robot.get_state()

        if not state:
            return

        twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
        twist.clamp(self.max_velocity, 0.5)

        # Compute Avoidance-DS
        position = state.ee_state.get_position()
        self.update_center_linear()
        desired_velocity = self.dynamics.evaluate(position)

        self.inital_velocity_publisher.publish(position, desired_velocity)
        self.initial_trajectory.publish(position)

        tic = time.perf_counter()
        # desired_velocity = self.avoider.evaluate(position)
        desired_velocity = self.dynamics.evaluate(position)
        toc = time.perf_counter()
        print(f"Timer took: {toc - tic:0.4f} s")
        self.rotated_velocity_publisher.publish(position, desired_velocity)

        # print("position", position)
        # print("attractr", self.dynamics.pose.position)

        # One time-step of the base-system.
        if np.linalg.norm(desired_velocity) > self.max_velocity:
            desired_velocity = (
                desired_velocity / np.linalg.norm(desired_velocity) * self.max_velocity
            )

        twist.set_linear_velocity(desired_velocity)
        # self.command_handler.update_state(state)
        self.command_handler.update_from_cartesian_twist(twist, state)
        # self.command_handler.limit_joint_velocity(0.01)
        self.robot.send_command(self.command_handler.get_command())


if __name__ == "__main__":
    print("Starting CartesianTwist controller ...")
    rclpy.init()
    # robot_interface = RobotInterface("*:1601", "*:1602")
    robot_interface = RobotInterface.from_id(17)

    controller = NonlinearAvoidanceController(
        # robot=robot_interface, freq=100, is_simulation=False
        robot=robot_interface,
        freq=50,
        is_simulation=False,
    )

    try:
        print("Startinit.")
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
