#!/usr/bin/env python3

import numpy as np
import rospy
import state_representation as sr
from controllers import create_cartesian_controller, CONTROLLER_TYPE
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM_TYPE
from network_interfaces.control_type import ControlType
from network_interfaces.zmq.network import CommandMessage
from learning_safety_margin.robot_interface import RobotInterface

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def convert_joint_state_msg(state):
    # Convert sr.joint_state msg to ros JointState msg
    header = Header()
    header.stamp = rospy.get_rostime()
    #header.frame_id = state.get_reference_frame() # is this important ??
    names = state.joint_state.get_names()
    pos = state.joint_state.get_positions()
    vel = state.joint_state.get_velocities()
    effort = state.joint_state.get_torques()
    msg = JointState(header, names, pos, vel, effort)

    return msg


def control_loop(robot, freq):

    # create publisher
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    target_set = False
    command = CommandMessage()
    command.control_type = [ControlType.EFFORT.value]

    ds = create_cartesian_ds(DYNAMICAL_SYSTEM_TYPE.POINT_ATTRACTOR)
    ds.set_parameter_value("gain", [0., 0., 0., 10., 10., 10.], sr.ParameterType.DOUBLE_ARRAY)

    ctrl = create_cartesian_controller(CONTROLLER_TYPE.COMPLIANT_TWIST)
    ctrl.set_parameter_value("linear_principle_damping", 0., sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("linear_orthogonal_damping", 0., sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("angular_stiffness", 5., sr.ParameterType.DOUBLE)
    ctrl.set_parameter_value("angular_damping", 5., sr.ParameterType.DOUBLE)

    print_count = 0

    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        state = robot.get_state()

        if not state:
            continue

        # debug print
        if print_count % freq == 0:
            print("EEF position: ", state.ee_state.get_position())
            # print("Joint positions: ", state.joint_state.get_positions())
            # print("EEF orientation: ", state.ee_state.get_orientation())
        print_count += 1

        if not target_set:
            target = sr.CartesianPose(state.ee_state.get_name(), np.array([.5, .0, .5]), np.array([0., 1., 0., 0.]),
                                      state.ee_state.get_reference_frame())
            ds.set_parameter_value("attractor", target, sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE)
            target_set = True
        else:
            twist = sr.CartesianTwist(ds.evaluate(state.ee_state))
            twist.clamp(.25, .5)
            # print("Desired Angular Velocities: ", twist.data()[3:6])
            command_torques = sr.JointTorques(ctrl.compute_command(twist, state.ee_state, state.jacobian))
            command.joint_state = state.joint_state
            # print("Command:", command.joint_state.get_torques())
            command.joint_state.set_torques(command_torques.get_torques())
            robot.send_command(command)
            #print("Command:", command)

        # Publish joint states for recording
        pub.publish(convert_joint_state_msg(state))

        rate.sleep()


if __name__ == '__main__':

    rospy.init_node("test", anonymous=True)

    print("starting Idle controller")

    robot_interface = RobotInterface("*:1701", "*:1702")
    control_loop(robot_interface, 500)
