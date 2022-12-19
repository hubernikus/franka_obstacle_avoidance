"""
Run and publish to pybullet together with robot.
To run the script, first launch the pybullet-simulation:

docker-terminal 1:
rviz

docker-terminal 2:
python example_optitrack_rviz.py
"""
import time

import numpy as np

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from franka_avoidance.optitrack_container import OptitrackContainer
from franka_avoidance.rviz_handler import RvizHandler


def main(frequency: float = 10):
    delta_time = 1.0 / frequency
    print("Start single obstacle node.")
    obstacles = OptitrackContainer(use_optitrack=True)
    obstacles.append(
        Ellipse(
            center_position=np.array([0.3, 2, 0]), axes_length=np.array([0.3, 0.3, 0.3])
        ),
        obstacle_id=27,
    )

    obstacles.visualization_handler = RvizHandler(obstacles)

    try:
        while True:
            obstacles.update()
            time.sleep(delta_time)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected.")

    obstacles.shutdown()


if (__name__) == "__main__":
    main()
