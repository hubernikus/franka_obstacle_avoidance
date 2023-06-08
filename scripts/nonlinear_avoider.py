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
from geometry_msgs.msg import TwistStamped, PoseStamped

# LASA Libraries
import state_representation as sr
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE

# Custom libraries
from vartools.states import Pose

from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid

# from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from nonlinear_avoidance.dynamics import DynamicDynamics
from nonlinear_avoidance.dynamics import SimpleCircularDynamics
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
from franka_avoidance.rviz_handler import RvizHandler
from franka_avoidance.velocity_publisher import VelocityPublisher
from franka_avoidance.trajectory_publisher import TrajectoryPublisher
from franka_avoidance.optitrack_container import OptitrackContainer
from franka_avoidance.controller_interface import (
    VelocityCommandHandler,
    ImpedanceCommandHandler,
)


def create_conveyer_obstacles(margin_absolut=0.1, distance_scaling=10.0):
    print(f"Create environment with")
    print(f"margin={margin_absolut}")
    print(f"scaling={distance_scaling}")

    optitrack_obstacles = OptitrackContainer(use_optitrack=True)

    # Conveyer Belt [static]
    conveyer_belt = MultiObstacle(Pose.create_trivial(dimension=3))
    conveyer_belt.set_root(
        Cuboid(
            center_position=np.array([0.5, 0.0, -0.25]),
            axes_length=np.array([0.5, 2.5, 0.8]),
            margin_absolut=margin_absolut,
            distance_scaling=distance_scaling,
        )
    )

    optitrack_obstacles.append(conveyer_belt, obstacle_id=-2)

    box1 = MultiObstacle(Pose.create_trivial(dimension=3))
    box1.set_root(
        Cuboid(
            center_position=np.array([0.0, 0, -0.06]),
            axes_length=np.array([0.16, 0.16, 0.16]),
            margin_absolut=margin_absolut,
            distance_scaling=distance_scaling,
        )
    )
    box1[-1].set_reference_point(np.array([0.0, 0.0, -0.08]), in_global_frame=False)
    optitrack_obstacles.append(box1, obstacle_id=1025)

    box2 = MultiObstacle(Pose.create_trivial(dimension=3))
    box2.set_root(
        Cuboid(
            center_position=np.array([0.0, 0, -0.12]),
            axes_length=np.array([0.26, 0.37, 0.24]),
            margin_absolut=margin_absolut,
            distance_scaling=distance_scaling,
        )
    )
    box2[-1].set_reference_point(np.array([0.0, 0.0, -0.12]), in_global_frame=False)
    optitrack_obstacles.append(box2, obstacle_id=1026)

    # optitrack_obstacles.visualization_handler = RvizHandler(optitrack_obstacles)
    return optitrack_obstacles


def create_circular_conveyer_dynamics():
    center_pose = Pose(
        np.array([0.5, 0.0, 0.3]),
        orientation=Rotation.from_euler("x", 0.0),
    )
    dynamics = SimpleCircularDynamics(pose=center_pose, radius=0.1)
    return dynamics

    # def create_circular_dynamics():
    #     # Create circular - circular dynamics
    #     center_pose = Pose(
    #         np.array([0.9, -0.2, 0.8]),
    #         orientation=Rotation.from_euler("y", -math.pi / 2),
    #     )
    #
    #     pose_base = copy.deepcopy(center_pose)
    #     pose_base.position = pose_base.position
    #     dynamics = SimpleCircularDynamics(pose=pose_base, radius=0.2)
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

    return dynamics


def create_rviz_handler(optitrack_obstacles):
    rviz_handler = RvizHandler(base_frame="panda_link0")
    cardboard_color = (0.8, 0.4, 0.4, 1.0)
    colors = [(0.4, 0.5, 0.4, 1.0), cardboard_color, cardboard_color]
    # rviz_handler = RvizHandler(base_frame="world")
    rviz_handler.create_multi_obstacles(
        optitrack_obstacles, optitrack_obstacles.obstacle_ids, colors=colors
    )
    return rviz_handler


def create_avoider(dynamics, optitrack_obstacles):
    # Transform to MultiObbstacleContainer and create avoider
    container = MultiObstacleContainer()
    for obs in optitrack_obstacles:
        container.append(obs)

    avoider = MultiObstacleAvoider(
        initial_dynamics=dynamics,
        obstacle_container=container,
        create_convergence_dynamics=True,
    )
    return avoider


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
    name: str,
    reference_frame: str,
    position: np.ndarray,
    orientation: Optional[np.ndarray] = None,
):
    if orientation is None:
        # Default pose is the robot pointing down
        orientation = np.array([0.0, 1.0, 0.0, 0.0])
    else:
        # make sure it's normalized
        orientation = orientation / np.linalg.norm(orientation)

    new_pose = sr.CartesianPose(
        # state.ee_state.get_name(),
        name,
        position,
        orientation,
        # state.ee_state.get_reference_frame(),
        reference_frame=reference_frame,
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
        use_twist_repeater: bool = False,
        target: Optional[sr.CartesianPose] = None,
        optitrack_obstacles: Optional[OptitrackContainer] = None,
        robot_frame: str = "panda_link0",
    ):
        super().__init__(node_name)
        self.robot = robot
        self.rate = self.create_rate(freq)
        period = 1.0 / freq

        self.robot_frame = robot_frame

        # self.max_linear_velocity = 0.25
        # self.max_angular_velocity = 0.1
        self.max_linear_velocity = 0.15
        self.max_angular_velocity = 0.1

        self.joint_robot = FrankaJointSpace()

        if is_velocity_controller:
            self.command_handler = VelocityCommandHandler()
        # Otherwise -> CartesianTwist
        elif is_simulation:
            self.command_handler = ImpedanceCommandHandler.create_simulation()
        elif use_twist_repeater:
            self.publisher_twist = self.create_publisher(
                TwistStamped, "/twist_repeater", 5
            )
            self.ee_pose = None
            self.subscriber_pose = self.create_subscription(
                PoseStamped, "/ee_pose", self.pose_callback, 5
            )
            while self.ee_pose is None and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=period)
                print("Waiting for ee_pose")

        else:
            print("Doing real-world impedance")
            self.command_handler = ImpedanceCommandHandler.create_realworld()
        # TODO: use enum to save mode (?)
        self.use_twist_repeater = use_twist_repeater

        # Get robot state to set up the target in the same frame
        state = None
        while (
            self.robot is not None
            and not (state := self.robot.get_state())
            and rclpy.ok()
        ):
            print("Awaiting first robot-state.")

        if target is None:
            if state is None:
                target = create_cartesian_pose(
                    reference_frame=robot_frame,
                    name="target",
                    position=np.array([0.3, 0.1, 0.8]),
                    orientation=None,
                    # orientation=np.array([0.0, 1.0, 0.0, 0.0]),
                )

            else:
                target = create_cartesian_pose(
                    name=state.ee_state.get_name(),
                    reference_frame=state.ee_state.get_reference_frame(),
                    position=np.array([0.3, 0.1, 0.8]),
                    orientation=None,
                    # orientation=np.array([0.0, 1.0, 0.0, 0.0]),
                )

        self.ds = create_target_ds(target)
        # self.create_circular_dynamics()
        self.dynamics = create_circular_conveyer_dynamics()
        self.it_dynamics_center = 0
        # self.create_obstacle_environment(margin_absolut=0.2)
        # self.human_with_limbs = create_flexilimb_human()

        self.optitrack_obstacles = optitrack_obstacles
        self.rviz_handler = create_rviz_handler(self.optitrack_obstacles)
        self.avoider = create_avoider(self.dynamics, self.optitrack_obstacles)

        self.inital_velocity_publisher = VelocityPublisher("initial", robot_frame)
        self.rotated_velocity_publisher = VelocityPublisher("rotated", robot_frame)
        self.initial_trajectory = TrajectoryPublisher(
            self.dynamics.evaluate, "initial", robot_frame
        )
        self.avoider_trajectory = TrajectoryPublisher(
            self.avoider.evaluate_sequence, "avoider", robot_frame
        )

        self.timer = self.create_timer(period, self.controller_callback)
        print("Finish initialization.")

    def update_center_linear(self, period: int = 1000) -> None:
        center1 = np.array([0.4, 0.0])
        center2 = np.array([0.8, 0.0])

        self.it_dynamics_center += 1

        progress = self.it_dynamics_center % period
        progress = abs(progress - period / 2) / period

        center = center1 * (1 - progress) + center2 * progress
        self.dynamics.pose.position[:2] = center

        # TODO: update additional settings / centers if necessary (!)

    def create_obstacle_environment(self, margin_absolut: float) -> None:
        obstacle1 = MultiObstacle()
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

    def publish_twist(self, twist) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_frame

        linear = twist.get_linear_velocity()
        msg.twist.linear.x = linear[0]
        msg.twist.linear.y = linear[1]
        msg.twist.linear.z = linear[2]

        angular = twist.get_angular_velocity()
        msg.twist.angular.x = angular[0]
        msg.twist.angular.y = angular[1]
        msg.twist.angular.z = angular[2]

        self.publisher_twist.publish(msg)

    def pose_callback(self, msg: PoseStamped) -> None:
        # self.ee_position = np.array(
        #     [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # )
        self.ee_pose = msg.pose

    def controller_callback(self) -> None:
        tic_loop = time.perf_counter()

        if not self.use_twist_repeater:
            state = self.robot.get_state()
            if not state:
                return

            ee_state = state.ee_state

        else:
            ee_state = sr.CartesianPose(
                name="current", reference_frame=self.robot_frame
            )

            ee_state.set_position(
                self.ee_pose.position.x,
                self.ee_pose.position.y,
                self.ee_pose.position.z,
            )

            ee_state.set_orientation(
                [
                    self.ee_pose.orientation.w,
                    self.ee_pose.orientation.x,
                    self.ee_pose.orientation.y,
                    self.ee_pose.orientation.z,
                ]
            )

        if self.optitrack_obstacles is not None:
            self.optitrack_obstacles.update()
            # self.avoider.obstacle_container.update_obstacle_pose()
            if self.rviz_handler is not None:
                self.rviz_handler.update_multi_obstacle(
                    self.optitrack_obstacles, self.optitrack_obstacles.obstacle_ids
                )

        if not self.use_twist_repeater:
            ee_position = state.ee_state.get_position()
        else:
            ee_position = np.array(
                [
                    self.ee_pose.position.x,
                    self.ee_pose.position.y,
                    self.ee_pose.position.z,
                ]
            )

        # self.update_center_linear()

        desired_velocity = self.dynamics.evaluate(ee_position)
        self.inital_velocity_publisher.publish(ee_position, desired_velocity)
        self.initial_trajectory.publish(ee_position)
        # (! WARNING) This might slow down the control loop
        self.avoider_trajectory.publish(ee_position)

        tic = time.perf_counter()
        desired_velocity = self.avoider.evaluate_sequence(ee_position)
        # time.sleep(0.03)
        toc = time.perf_counter()
        print(f"Avoider took: {toc - tic:0.4f} s")

        print_states = True
        if print_states:
            print("position", ee_position)
            print("desired_velocity", desired_velocity)

        # DON"T MOVE (
        desired_velocity = np.zeros(3)
        # Transfer to twist message
        twist = sr.CartesianTwist(self.ds.evaluate(ee_state))
        twist.set_linear_velocity(desired_velocity)
        twist.clamp(self.max_linear_velocity, self.max_angular_velocity)

        self.rotated_velocity_publisher.publish(ee_position, desired_velocity)

        if self.use_twist_repeater:
            self.publish_twist(twist)
        else:
            twist.set_linear_velocity(desired_velocity)
            self.command_handler.update_from_cartesian_twist(twist, state)
            self.robot.send_command(self.command_handler.get_command())

        toc_loop = time.perf_counter()
        # breakpoint()
        print(f"Full loop took: {toc_loop - tic_loop:0.4f} s")


if __name__ == "__main__":
    print("Starting CartesianTwist controller ...")
    rclpy.init()

    optitrack_obstacles = create_conveyer_obstacles()
    use_twist_repeater = True
    if use_twist_repeater:
        robot_interface = None
    else:
        robot_interface = RobotInterface.from_id(17)

    controller = NonlinearAvoidanceController(
        # robot=robot_interface, freq=100, is_simulation=False
        robot=robot_interface,
        freq=50,
        # is_simulation=False,
        use_twist_repeater=use_twist_repeater,
        optitrack_obstacles=optitrack_obstacles,
    )

    try:
        print("Startinit.")
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
