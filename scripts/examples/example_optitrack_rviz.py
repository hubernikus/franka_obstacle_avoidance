"""
Run and publish to pybullet together with robot.
To run the script, first launch the pybullet-simulation:
docker-terminal 1:
rviz
docker-terminal 2:
python example_optitrack_rviz.py
"""
import time

import rclpy
from rclpy.node import Node

import numpy as np

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from franka_avoidance.optitrack_container import OptitrackContainer
from franka_avoidance.rviz_handler import RvizHandler

# Custom libraries
from franka_avoidance.robot_interface import RobotZmqInterface as RobotInterface


class OptitrackController(Node):
    def __init__(
        self,
        robot: RobotInterface,
        obstacles,
        frequency: float,
    ):
        self.robot = robot
        self.obstacles = obstacles

        self.delta_time = 1.0 / frequency

    def run(self):

        while True:
            self.obstacles.update()
            state = self.robot.get_state()

            time.sleep(self.delta_time)


def main(frequency: float = 10):
    print("Start node.")
    rclpy.init()

    robot_interface = RobotInterface("*:1601", "*:1602")

    delta_time = 1.0 / frequency
    print("Start single obstacle node.")
    obstacles = OptitrackContainer(use_optitrack=True, update_frequency=frequency)
    obstacles.append(
        Ellipse(
            center_position=np.array([0.0, 0, -0.1]),
            axes_length=np.array([0.3, 0.3, 0.3]),
        ),
        obstacle_id=27,
    )

    obstacles.visualization_handler = RvizHandler(obstacles)

    controller = OptitrackController(
        robot=robot_interface, obstacles=obstacles, frequency=frequency
    )
    try:
        controller.run()
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected.")

    obstacles.shutdown()


if (__name__) == "__main__":
    main()
