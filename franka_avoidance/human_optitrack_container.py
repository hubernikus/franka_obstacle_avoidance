"""
Create a Human which updates using optitrack.
"""
import numpy as np

from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from roam.multi_body_human import MultiBodyObstacle

from franka_avoidance.optitrack_interface import OptitrackInterface
from franka_avoidance.optitrack_interface import SimpleRobot
from franka_avoidance.rviz_handler import RvizHandler


def create_optitrack_human() -> MultiBodyObstacle:
    """Factory function which gives human based on specific setup."""
    dimension = 3

    new_human = MultiBodyObstacle(
        visualization_handler=RvizHandler(base_frame="panda_link0"),
        pose_updater=OptitrackInterface(
            robot_id=-1
        ),  # Robot is not defined here anymore...
        robot=SimpleRobot(robot_id=16),
    )

    # Optitrack id's
    id_body = 101
    id_upperarm2 = 102
    id_lowerarm2 = 103
    id_upperarm1 = 104
    id_lowerarm1 = 105

    upper_arm_axes = [0.18, 0.18, 0.5]
    lower_arm_axes = [0.14, 0.14, 0.4]
    head_dimension = [0.2, 0.15, 0.3]

    new_human.set_root(
        Cuboid(axes_length=[0.15, 0.4, 0.5], center_position=np.zeros(dimension)),
        name="body",
        update_id=id_body,
    )
    new_human[-1].set_reference_point(np.array([0, 0, -0.2]), in_global_frame=False)

    new_human.add_component(
        Ellipse(axes_length=[0.12, 0.15, 0.4], center_position=np.zeros(dimension)),
        name="neck",
        update_id=None,
        parent_name="body",
        reference_position=[0.0, 0.0, -0.07],
        parent_reference_position=[0.0, 0.0, 0.25],
    )

    new_human.add_component(
        Ellipse(axes_length=[0.2, 0.22, 0.3], center_position=np.zeros(dimension)),
        name="head",
        update_id=None,
        parent_name="neck",
        reference_position=[0.0, 0.0, 0.0],
        parent_reference_position=[0.0, 0.0, 0.07],
    )

    new_human.add_component(
        Ellipse(axes_length=upper_arm_axes, center_position=np.zeros(dimension)),
        name="upperarm1",
        update_id=id_upperarm1,
        parent_name="body",
        reference_position=[0, 0, -0.2],
        parent_reference_position=[0.0, 0.18, 0.2],
    )

    new_human.add_component(
        Ellipse(axes_length=lower_arm_axes, center_position=np.zeros(dimension)),
        name="lowerarm1",
        update_id=id_lowerarm1,
        parent_name="upperarm1",
        reference_position=[0.0, 0, -0.18],
        parent_reference_position=[0.0, 0, 0.2],
    )

    new_human.add_component(
        Ellipse(axes_length=upper_arm_axes, center_position=np.zeros(dimension)),
        name="upperarm2",
        update_id=id_upperarm2,
        parent_name="body",
        reference_position=[0.0, 0, -0.2],
        parent_reference_position=[0.0, -0.18, 0.2],
    )

    new_human.add_component(
        Ellipse(axes_length=lower_arm_axes, center_position=np.zeros(dimension)),
        name="lowerarm2",
        update_id=id_lowerarm2,
        parent_name="upperarm2",
        reference_position=[0.0, 0, -0.18],
        parent_reference_position=[0.0, 0, 0.2],
    )

    return new_human


def plot_human_obstacle():
    fig = plt.figure()
    ax = plt.axes(projection="3d")

    human_with_limbs = HumanTrackContainer.create_optitrack_human()
    plot_3d_cuboid(ax, human_with_limbs.body)
    # plot_3d_ellipsoid(ax, human_with_limbs.body)
    plot_3d_ellipsoid(ax, human_with_limbs.upperarm0)
    ax.axis("equal")


def main():
    # Put everything in main to avoid import outside of 'test'.
    import rclpy
    from rclpy.node import Node

    class HumanVisualizer(Node):
        def __init__(self, frequency: float = 100.0) -> None:
            super().__init__("human_visualizer")
            self.human_with_limbs = MultiBodyObstacle.create_optitrack_human()
            period = 1.0 / frequency

            self.timer = self.create_timer(period, self.visualizer_callback)

        def visualizer_callback(self):
            self.human_with_limbs.update()

    print("Starting HumanVisualizer ...")
    rclpy.init()
    visualizer = HumanVisualizer()

    try:
        rclpy.spin(visualizer)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if (__name__) == "__main__":
    import matplotlib.pyplot as plt

    plt.ion()
    # plot_human_obstale()

    main()
