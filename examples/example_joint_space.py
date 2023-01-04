#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

import threading

# LASA Libraries
import state_representation as sr
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage

# Custom libraries
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface


class JointSpaceController(Node):
    def __init__(self, robot, freq: float = 100, node_name="joint_controller"):
        super().__init__(node_name)
        self.robot = robot
        self.rate = self.create_rate(freq)

        self.ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds.set_parameter_value(
            "gain", [50.0, 50.0, 50.0, 10.0, 10.0, 10.0], sr.ParameterType.DOUBLE_ARRAY
        )

    def run(self):
        target_set = False

        while rclpy.ok():
            self.command = CommandMessage()
            self.command.control_type = [ControlType.VELOCITY.value]

            state = self.robot.get_state()
            if not state:
                continue
            if not target_set:
                target = sr.CartesianPose(
                    state.ee_state.get_name(),
                    np.array([0.6, 0.3, 0.5]),
                    np.array([0.0, 1.0, 0.0, 0.0]),
                    state.ee_state.get_reference_frame(),
                )
                self.ds.set_parameter_value(
                    "attractor",
                    target,
                    sr.ParameterType.STATE,
                    sr.StateType.CARTESIAN_POSE,
                )
                target_set = True
            else:
                twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
                twist.clamp(0.1, 0.3)
                # print('tor', self.command.joint_state.get_torques())
                self.command.joint_state = state.joint_state
                self.command.joint_state.set_velocities(
                    np.linalg.lstsq(state.jacobian.data(), twist.get_twist())[0]
                )
                self.robot.send_command(self.command)
                # print('vel', self.command.joint_state.get_velocities())
                print('tor', self.command.joint_state.get_torques())
                self.rate.sleep()


if __name__ == "__main__":
    print("Starting Joint Space....")
    rclpy.init()
    # rospy.init_node("test", anonymous=True)
    robot_interface = RobotInterface("*:1601", "*:1602")

    # Spin in a separate thread
    controller = JointSpaceController(robot=robot_interface, freq=500)

    thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    thread.start()

    try:
        controller.run()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
