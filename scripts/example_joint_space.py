#!/usr/bin/env python3
import numpy as np

import rospy

# LASA Libraries
import state_representation as sr
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage

# Custom Libraries
from robot_interface import RobotInterface


class JointSpaceController:
    def __init__(self, robot, freq: float = 100):
        self.command = CommandMessage()
        self.command.control_type = [ControlType.VELOCITY.value]

        self.ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds.set_parameter_value(
            "gain", [50.0, 50.0, 50.0, 10.0, 10.0, 10.0], sr.ParameterType.DOUBLE_ARRAY
        )

        self.rate = rospy.Rate(freq)
        self.robot = robot

    def run(self):
        target_set = False

        while not rospy.is_shutdown():
            state = self.robot.get_state()
            if not state:
                continue
            if not target_set:
                target = sr.CartesianPose(
                    state.ee_state.get_name(),
                    np.array([0.6, -0.3, 0.5]),
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
                twist = sr.CartesianTwist(ds.evaluate(state.ee_state))
                twist.clamp(0.25, 0.5)
                self.command.joint_state = state.joint_state
                self.command.joint_state.set_velocities(
                    np.linalg.lstsq(state.jacobian.data(), twist.get_twist())[0]
                )
                self.robot.send_command(command)
                self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("test", anonymous=True)

    robot_interface = RobotInterface("*:1601", "*:1602")

    controller = JointSpaceController(robot=robot_interface)
    controller.run(500)
