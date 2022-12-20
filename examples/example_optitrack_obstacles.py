"""
Example how to use the optitrack container together with the rviz publisher.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from franka_avoidance.optitrack_container import OptitrackContainer
from franka_avoidance.rviz_handler import RvizHandler

from network_interfaces.zmq.network import CommandMessage

# Custom libraries
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface


class VelocityAvoidanceController(Node):
    def __init__(
        self,
        robot,
        freq: float = 100,
        node_name: str = "velocity_avoidance_controller",
        obstacles: OptitrackContainer = None,
    ):
        super().__init__(node_name)

        self.robot = robot
        self.rate = self.create_rate(freq)

        self.command = CommandMessage()

    def run(self):
        while rclpy.ok():
            state = self.robot.get_state()
            if not state:
                continue


if __name__ == "__main__":
    print("Start node.")
    rclpy.init()

    robot = robot_interface = RobotInterface("*:1601", "*:1602")

    obstacles = OptitrackContainer(use_optitrack=False)
    obstacles.append(
        Ellipse(
            center_position=np.array([0.3, 2, 0]), axes_length=np.array([0.3, 0.3, 0.3])
        ),
        obstacle_id=0,
    )
    obstacles.visualization_handler = RvizHandler(obstacles)

    controller = VelocityAvoidanceController(robot, freq=100, obstacles=obstacles)
    try:
        controller.run()
    except:
        print("Shutting down.")

    rclpy.shutdown()
