from __future__ import annotations  # Self typing

from dataclasses import dataclass, field
from typing import Optional
import warnings

import numpy as np
import numpy.typing as npt
from numpy import linalg

from scipy.spatial.transform import Rotation

import networkx as nx

from dynamic_obstacle_avoidance.containers import ObstacleContainer

from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from franka_avoidance.rviz_handler import RvizHandler
from franka_avoidance.optitrack_interface import SimpleRobot, RigidBody
from franka_avoidance.optitrack_interface import OptitrackInterface

from franka_avoidance.state_filters import PositionFilter, SimpleOrientationFilter


def plot_3d_cuboid(ax, cube: Cuboid, color="green"):
    # TODO: include orientation
    axis = cube.axes_length
    orientation = cube.orientation

    pos_ranges = np.array(
        [
            cube.center_position - axis / 2.0,
            cube.center_position + axis / 2.0,
        ]
    ).T
    posx = pos_ranges[0, :]
    posy = pos_ranges[1, :]
    posz = pos_ranges[2, :]

    # Define the vertices of the cube
    for ii in posx:
        for jj in posy:
            ax.plot([ii, ii], [jj, jj], posz, color=color, marker="o")

    for ii in posx:
        for jj in posz:
            ax.plot([ii, ii], posy, [jj, jj], color=color, marker="o")

    for ii in posy:
        for jj in posz:
            ax.plot(posx, [ii, ii], [jj, jj], color=color, marker="o")


def plot_3d_ellipsoid(ax, ellipse: Ellipse):
    # TODO: inclde orientation?

    # your ellispsoid and center in matrix form
    diag_axes = np.diag(ellipse.axes_length)
    # dimension = 3
    # A = np.eye(dimension)
    # for dd in range(dimension):
    #     A[dd, :] =
    A = diag_axes

    # find the rotation matrix and radii of the axes
    U, s, rotation = linalg.svd(A)
    # radii = 1.0 / np.sqrt(s)
    radii = ellipse.axes_length / 2.0

    # now carry on with EOL's answer
    u = np.linspace(0.0, 2.0 * np.pi, 100)
    v = np.linspace(0.0, np.pi, 100)

    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))

    for i in range(len(x)):
        for j in range(len(x)):
            [x[i, j], y[i, j], z[i, j]] = (
                np.dot([x[i, j], y[i, j], z[i, j]], rotation) + ellipse.center_position
            )

    ax.plot_wireframe(x, y, z, rstride=4, cstride=4, color="b", alpha=0.2)


# class HumanTrackContainer(Obstacle):
class HumanTrackContainer:
    dimension = 3

    def __init__(self, update_frequency: float = 100.0, visualization_handler=None):
        # super().__init__(center_position=np.zeros(3))

        self._obstacle_list: list[Obstacle] = []

        self._graph = nx.DiGraph()
        self.robot = SimpleRobot(robot_id=16)
        self.optitrack_interface = OptitrackInterface()

        self._id_counter = 0

        self.position_filters = []
        self.orientation_filters = []
        self.update_frequency = update_frequency

        self.visualization_handler = visualization_handler

    def __getitem__(self, key) -> Obstacle:
        return self._obstacle_list[key]

    def __setitem__(self, key: int, value: Obstacle) -> None:
        self._obstacle_list[key] = value

    def get_obstacle_id_from_name(self, name: str) -> int:
        return [x for x, y in self._graph.nodes(data=True) if y["name"] == name][0]

    def get_obstacle_id_from_optitrackid(self, opt_id: int) -> int:
        return [
            x for x, y in self._graph.nodes(data=True) if y["optitrack_id"] == opt_id
        ][0]

    def get_component(self, idx: int) -> Obstacle:
        return self._obstacle_list[idx]

    def get_parent_idx(self, idx_obs: int) -> Optional[int]:
        if idx_obs == self.root_idx:
            return None
        else:
            return list(self._graph.predecessors(idx_obs))[0]

    @property
    def root_id(self) -> int:
        return self.root_idx

    @property
    def n_components(self) -> int:
        return len(self._obstacle_list)

    def set_root(
        self,
        obstacle: Obstacle,
        name: str,
        optitrack_id: Optional[int] = None,
    ):
        self._obstacle_list.append(obstacle)
        self._graph.add_node(
            self._id_counter,
            name=name,
            optitrack_id=optitrack_id,
            references_children=[],
            indeces_children=[],
        )

        self.create_filters(is_updating=(not optitrack_id is None))

        self._id_counter += 1
        self.root_idx = 0

    def create_filters(self, is_updating: bool):
        if is_updating:
            self.position_filters.append(
                PositionFilter(
                    update_frequency=self.update_frequency,
                    initial_position=np.zeros(3),
                )
            )
            self.orientation_filters.append(
                SimpleOrientationFilter(
                    update_frequency=self.update_frequency,
                    initial_orientation=Rotation.from_euler("x", 0),
                )
            )
        else:
            self.position_filters.append(None)
            self.orientation_filters.append(None)

    def get_optitrack_id(self, idx_node: int) -> int:
        return self._graph.nodes[idx_node]["optitrack_id"]

    def add_limb(
        self,
        obstacle: Obstacle,
        name: str,
        reference_position: npt.ArrayLike,
        parent_name: str,
        parent_reference_position: npt.ArrayLike,
        optitrack_id: Optional[int] = None,
    ):
        self._obstacle_list.append(obstacle)
        parent_ind = self.get_obstacle_id_from_name(parent_name)

        self._graph.add_node(
            self._id_counter,
            name=name,
            optitrack_id=optitrack_id,
            local_reference=np.array(reference_position),
            indeces_children=[],
            references_children=[],
        )
        self._graph.nodes[parent_ind]["references_children"].append(
            np.array(parent_reference_position)
        )
        self._graph.nodes[parent_ind]["indeces_children"].append(self._id_counter)

        self._graph.add_edge(parent_ind, self._id_counter)

        self.create_filters(is_updating=(not optitrack_id is None))
        self._id_counter += 1

    def update_using_optitrack(self):
        optitrack_measures = self.optitrack_interface.get_messages()
        indeces_measures = [oo.obs_id for oo in optitrack_measures]

        try:
            index_franka_list = indeces_measures.index(self.robot.optitrack_id)

        except ValueError:
            # Element not in list
            pass

        else:
            franka_object = optitrack_measures[index_franka_list]
            self.robot.position = franka_object.position
            self.robot.rotation = franka_object.rotation

        try:
            idx_measure = indeces_measures.index(self.root_idx)

        except ValueError:
            # Element not in list
            pass
        else:
            self.update_dynamic_obstacle(self.root_idx, optitrack_measures[idx_measure])

        obs_indeces = list(self._graph.successors(self.root_idx))
        it_node = 0
        while it_node < len(obs_indeces):
            idx_node = obs_indeces[it_node]
            obs_indeces = obs_indeces + list(self._graph.successors(idx_node))

            it_node += 1  # Iterate

            idx_optitrack = self._graph.nodes[idx_node]["optitrack_id"]
            try:
                idx_measure = indeces_measures.index(idx_optitrack)

            except ValueError:
                # Static opbstacle - no optitrack exists...
                # Update rotation
                idx_parent = list(self._graph.predecessors(idx_node))[0]
                self[idx_node].orientation = self[idx_parent].orientation
                self.align_position_with_parent(idx_node)

            else:
                self.update_dynamic_obstacle(idx_node, optitrack_measures[idx_measure])
                self.align_position_with_parent(idx_node)

                # Reset position filter
                self.position_filters[idx_node]._position = self[idx_node].pose.position

    def update_dynamic_obstacle(self, idx_obs: int, obs_measure: RigidBody):
        # Update position
        self.position_filters[idx_obs].run_once(obs_measure.position)
        self.orientation_filters[idx_obs].run_once(obs_measure.rotation)

        self[idx_obs].pose.position = self.robot.pose.transform_position_to_relative(
            self.position_filters[idx_obs].position
        )
        self[
            idx_obs
        ].pose.orientation = self.robot.pose.transform_orientation_to_relative(
            self.orientation_filters[idx_obs].rotation
        )
        self[
            idx_obs
        ].linear_velocity = self.robot.pose.transform_linear_velocity_to_relative(
            self.position_filters[idx_obs].velocity
        )

    def align_position_with_parent(self, idx_obs: int):
        """Update obstacle with respect to the movement of the body-parts (limbs)
        under the assumption of FULL-TRUST(!) to the orientation."""
        idx_parent = list(self._graph.predecessors(idx_obs))[0]
        reference_obstacle = self[idx_obs].pose.transform_position_from_relative(
            self._graph.nodes[idx_obs]["local_reference"]
        )

        idx_local_ref = self._graph.nodes[idx_parent]["indeces_children"].index(idx_obs)

        local_reference_parent = self._graph.nodes[idx_parent]["references_children"][
            idx_local_ref
        ]
        reference_parent = self[idx_parent].pose.transform_position_from_relative(
            local_reference_parent
        )

        delta_ref = reference_parent - reference_obstacle

        # Full believe in orientation (and parent)
        self[idx_obs].pose.position = self[idx_obs].pose.position + delta_ref

    def update(self):
        self.update_using_optitrack()

        self.robot.publish_robot_transform()
        self.visualization_handler.update(self._obstacle_list)

    @classmethod
    def create_optitrack_human(cls) -> Self:
        """Factory function which gives human based on specific setup."""
        visualization_handler = RvizHandler(base_frame="panda_link0")
        opti_human = cls(visualization_handler=visualization_handler)

        id_body = 101
        id_upperarm1 = 102
        id_lowerarm1 = 103
        id_upperarm2 = 104
        id_lowerarm2 = 105

        upper_arm_axes = [0.5, 0.18, 0.18]
        lower_arm_axes = [0.4, 0.14, 0.14]
        head_dimension = [0.2, 0.15, 0.3]

        opti_human.set_root(
            Cuboid(axes_length=[0.4, 0.15, 0.5], center_position=np.zeros(3)),
            name="body",
            optitrack_id=id_body,
        )
        opti_human.add_limb(
            Ellipse(axes_length=[0.12, 0.15, 0.4], center_position=np.zeros(3)),
            name="neck",
            optitrack_id=None,
            parent_name="body",
            reference_position=[0.0, 0.0, -0.07],
            parent_reference_position=[0.0, 0.0, 0.25],
        )

        opti_human.add_limb(
            Ellipse(axes_length=[0.2, 0.22, 0.3], center_position=np.zeros(3)),
            name="head",
            optitrack_id=None,
            parent_name="neck",
            reference_position=[0.0, 0.0, 0.0],
            parent_reference_position=[0.0, 0.0, 0.07],
        )

        opti_human.add_limb(
            Ellipse(axes_length=upper_arm_axes, center_position=np.zeros(3)),
            name="upperarm1",
            optitrack_id=id_upperarm1,
            parent_name="body",
            reference_position=[-0.2, 0.0, 0],
            parent_reference_position=[0.15, 0.0, 0.2],
        )

        opti_human.add_limb(
            Ellipse(axes_length=lower_arm_axes, center_position=np.zeros(3)),
            name="lowerarm1",
            optitrack_id=id_lowerarm1,
            parent_name="upperarm1",
            reference_position=[-0.18, 0.0, 0],
            parent_reference_position=[0.2, 0.0, 0],
        )

        opti_human.add_limb(
            Ellipse(axes_length=upper_arm_axes, center_position=np.zeros(3)),
            name="upperarm2",
            optitrack_id=id_upperarm2,
            parent_name="body",
            reference_position=[0.2, 0.0, 0],
            parent_reference_position=[-0.15, 0.0, 0.2],
        )

        opti_human.add_limb(
            Ellipse(axes_length=lower_arm_axes, center_position=np.zeros(3)),
            name="lowerarm2",
            optitrack_id=id_lowerarm2,
            parent_name="upperarm2",
            reference_position=[0.18, 0.0, 0],
            parent_reference_position=[-0.2, 0.0, 0],
        )

        return opti_human


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
            self.human_with_limbs = HumanTrackContainer.create_optitrack_human()
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
