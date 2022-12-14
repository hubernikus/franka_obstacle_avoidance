"""
Run and publish to pybullet together with robot.
"""
import time

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from franka_avoidance.optitrack_container import OptitrackContainer
from franka_avoidance.pybullet_handler import PybulletHandler


def main():
    obstacles = OptitrackContainer(use_optitrack=False)
    obstacles.append(
        Ellipse(
            center_position=np.array([0.3, 2, 0]), axes_length=np.array([0.3, 0.3, 0.3])
        ),
        obstacle_id=0,
    )

    obstacles.visualization_handler = PybulletHandler(obstacles)

    # try:
    for ii in range(100):
        obstacles.update_obstacles()
        time.sleep(0.2)
    # except:
    #     pass

    obstacles.shutdown()


if (__name__) == "__main__":
    main()
