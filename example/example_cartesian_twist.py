#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

import threading

import state_representation as sr
from controllers import create_cartesian_controller, CONTROLLER_TYPE
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage

# Custom libraries
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface


class TwistController(Node):
    def __init__(self, robot, freq: float = 100, node_name="twist_controller"):
        super().__init__(node_name)
        self.robot = robot
        self.rate = self.create_rate(freq)

        self.command = CommandMessage()
        self.command.control_type = [ControlType.EFFORT.value]

        self.ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
        self.ds.set_parameter_value(
            "gain", [50.0, 50.0, 50.0, 10.0, 10.0, 10.0], sr.ParameterType.DOUBLE_ARRAY
            )

        self.ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
        self.ctrl.set_parameter_value("linear_principle_damping", 1.0, sr.ParameterType.DOUBLE)
        self.ctrl.set_parameter_value("linear_orthogonal_damping", 1.0, sr.ParameterType.DOUBLE)
        self.ctrl.set_parameter_value("angular_stiffness", 0.5, sr.ParameterType.DOUBLE)
        self.ctrl.set_parameter_value("angular_damping", 0.5, sr.ParameterType.DOUBLE)

    def run(self):
        target_set = False

        while rclpy.ok():
            state = self.robot.get_state()
            if not state:
                continue
            if not target_set:
                target = sr.CartesianPose(
                    state.ee_state.get_name(),
                    np.array([0.3, 0.4, 0.5]),
                    np.array([0.0, 1.0, 0.0, 0.0]),
                    state.ee_state.get_reference_frame(),
                )
                self.ds.set_parameter_value(
                    "attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE
                )
                target_set = True
            else:
                twist = sr.CartesianTwist(self.ds.evaluate(state.ee_state))
                twist.clamp(0.25, 0.5)
                self.command_torques = sr.JointTorques(
                    self.ctrl.compute_command(twist, state.ee_state, state.jacobian)
                )
                self.command.joint_state = state.joint_state
                self.command.joint_state.set_torques(self.command_torques.get_torques())
                
                self.robot.send_command(self.command)

            self.rate.sleep()

            
if __name__ == "__main__":
    rclpy.init()
    # rospy.init_node("test", anonymous=True)
    robot_interface = RobotInterface("*:1601", "*:1602")

    # Spin in a separate thread
    controller = TwistController(robot=robot_interface, freq=500)

    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    try:
        controller.run()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
