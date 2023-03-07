from __future__ import annotations  # Self typing

from dataclasses import dataclass, field
from typing import Optional
import warnings

import numpy as np
import numpy.typing as npt
from numpy import linalg as LA

from scipy.spatial.transform import Rotation

import networkx as nx

from vartools.state_filters import PositionFilter, SimpleOrientationFilter
from vartools.states import ObjectPose

from dynamic_obstacle_avoidance.containers import ObstacleContainer
from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from roam.rigid_body import RigidBody
from roam.multi_obstacle_avoider import MultiObstacleAvoider
from roam.dynamics.circular_dynamics import SimpleCircularDynamics
from roam.dynamics.projected_rotation_dynamics import (
    ProjectedRotationDynamics,
)


def get_rotation_between_vectors(vec1: np.ndarray, vec2: np.ndarray) -> Rotation:
    if not (vec1_norm := np.linalg.norm(vec1)):
        return Rotation.from_quat([0, 0, 0, 1.0])
    vec1 = np.array(vec1) / vec1_norm

    if not (vec2_norm := np.linalg.norm(vec2)):
        return Rotation.from_quat([0, 0, 0, 1.0])
    vec2 = np.array(vec2) / vec2_norm

    rot_vec = np.cross(vec1, vec2)
    if not (rotvec_norm := np.linalg.norm(rot_vec)):
        return Rotation.from_quat([0, 0, 0, 1.0])

    rot_vec = rot_vec / rotvec_norm
    theta = np.arcsin(rotvec_norm)
    quat = np.hstack((rot_vec * np.cos(theta / 2.0), [np.sin(theta / 2.0)]))
    return Rotation.from_quat(quat)


@dataclass
class Joint:
    # If needed - a filter could be added here...
    id_optitrack: int
    position: np.ndarray
    # joint_name: str


class FlexiLimbHuman:
    dimension = 3

    def __init__(
        self,
        update_frequency: float = 100.0,
        visualization_handler=None,
        pose_updater=None,
        robot=None,
        id_ellbow_r: Optional[int] = None,
        id_wrist_r: Optional[int] = None,
        id_shoulder_r: Optional[int] = None,
        id_shoulder_l: Optional[int] = None,
    ):
        # super().__init__(center_position=np.zeros(3))

        self._obstacle_list: list[Obstacle] = []

        self._graph = nx.DiGraph()
        self.robot = robot

        # Pose updater (Optional) can be for example an OptitrackInterface
        self.pose_updater = pose_updater

        self._id_counter = 0

        self.position_filters = []
        self.orientation_filters = []
        self.update_frequency = update_frequency

        self.visualization_handler = visualization_handler

        self.joints: dict[str, Joint] = {}
        self.joints["ellbow_r"] = Joint(id_ellbow_r, np.zeros(self.dimension))
        self.joints["wrist_r"] = Joint(id_wrist_r, np.zeros(self.dimension))
        self.joints["shoulder_r"] = Joint(id_shoulder_r, np.zeros(self.dimension))
        self.joints["shoulder_l"] = Joint(id_shoulder_l, np.zeros(self.dimension))

        self.margin_upperarms: Optional[float] = None

    def __getitem__(self, key) -> Obstacle:
        return self._obstacle_list[key]

    def __setitem__(self, key: int, value: Obstacle) -> None:
        self._obstacle_list[key] = value

    def get_obstacle_id_from_name(self, name: str) -> int:
        return [x for x, y in self._graph.nodes(data=True) if y["name"] == name][0]

    def get_name(self, idx: int) -> str:
        return self._graph.nodes(data=True)[idx]["name"]

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
        update_id: Optional[int] = None,
    ):
        self._obstacle_list.append(obstacle)
        self._graph.add_node(
            self._id_counter,
            name=name,
            update_id=update_id,
            references_children=[],
            indeces_children=[],
        )

        self.create_filters(is_updating=(not update_id is None))

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

    def get_update_id(self, idx_node: int) -> int:
        return self._graph.nodes[idx_node]["update_id"]

    def add_component(
        self,
        obstacle: Obstacle,
        name: str,
        reference_position: npt.ArrayLike,
        parent_name: str,
        parent_reference_position: npt.ArrayLike,
        update_id: Optional[int] = None,
    ):
        reference_position = np.array(reference_position)
        obstacle.set_reference_point(reference_position, in_global_frame=False)
        self._obstacle_list.append(obstacle)
        parent_ind = self.get_obstacle_id_from_name(parent_name)

        self._graph.add_node(
            self._id_counter,
            name=name,
            update_id=update_id,
            local_reference=reference_position,
            indeces_children=[],
            references_children=[],
        )
        self._graph.nodes[parent_ind]["references_children"].append(
            np.array(parent_reference_position)
        )
        self._graph.nodes[parent_ind]["indeces_children"].append(self._id_counter)

        self._graph.add_edge(parent_ind, self._id_counter)

        self.create_filters(is_updating=(not update_id is None))
        self._id_counter += 1

    @property
    def optitrack_indeces(self):
        indeces_tree_opti = [-1] * self.n_components
        for ii in range(self.n_components):
            if self._graph.nodes[ii]["update_id"] is not None:
                indeces_tree_opti[ii] = self._graph.nodes[ii]["update_id"]

        return indeces_tree_opti

    def update_joints(self) -> None:
        for kk in self.joints.keys():
            try:
                idx_joint = self.indeces_measures.index(self.joints[kk].id_optitrack)

            except ValueError:
                # Element n ot in list
                continue

            self.joints[kk].position = self.robot.pose.transform_position_to_relative(
                self.new_object_poses[idx_joint].position
            )

    def update_robot(self):
        if self.robot is not None:
            return
        try:
            index_franka_list = self.indeces_measures.index(self.robot.optitrack_id)

        except ValueError:
            # Element not in list
            pass

        else:
            # So far: no filter for the robot (!)
            franka_object = self.new_object_poses[index_franka_list]

            self.robot.pose.position = franka_object.position
            self.robot.pose.rotation = franka_object.rotation

    def get_joint_position_to_parent(self, name: str) -> np.ndarray:
        idx_obs = self.get_obstacle_id_from_name(name)
        idx_parent = list(self._graph.predecessors(idx_obs))[0]
        idx_local_ref = self._graph.nodes[idx_parent]["indeces_children"].index(idx_obs)
        local_reference_parent = self._graph.nodes[idx_parent]["references_children"][
            idx_local_ref
        ]
        reference_parent = self[idx_parent].pose.transform_position_from_relative(
            local_reference_parent
        )
        return reference_parent

    def update_body(self):
        """Body is set to be in between shoulders."""
        idx_body = self.get_obstacle_id_from_name("body")
        self[idx_body].position = 0.5 * (
            self.joints["shoulder_l"].position + self.joints["shoulder_r"].position
        )
        self[idx_body].position[2] = 0.1  # No movement in z...

        self[idx_body].pose.orientation = get_rotation_between_vectors(
            [0, 1.0, 0],
            self.joints["shoulder_r"].position - self.joints["shoulder_l"].position,
        )
        zyx_rot = self[idx_body].pose.orientation.as_euler("zyx")
        self[idx_body].pose.orientation = Rotation.from_euler("z", zyx_rot[0])

        # The joints now have to be updated - based on the position
        self.joints["shoulder_l"].position = self.get_joint_position_to_parent(
            "upperarm_l"
        )
        self.joints["shoulder_r"].position = self.get_joint_position_to_parent(
            "upperarm_r"
        )

    def update_forearm(self):
        idx_body = self.get_obstacle_id_from_name("forearm_r")
        self[idx_body].position = 0.5 * (
            self.joints["wrist_r"].position + self.joints["ellbow_r"].position
        )

        ellbow_dir = self.joints["wrist_r"].position - self.joints["ellbow_r"].position
        self[idx_body].pose.orientation = get_rotation_between_vectors(
            [0.0, 0.0, 1.0],
            ellbow_dir,
        )

        # Update joints to be in-between
        if ellbow_norm := np.linalg.norm(ellbow_dir):
            ellbow_dir = ellbow_dir / ellbow_norm
        else:
            ellbow_dir = np.array([1.0, 0, 0])

        self.joints["ellbow_r"].position = (
            self[idx_body].position - self[idx_body].axes_length[0] * 0.5 * ellbow_dir
        )
        self.joints["wrist_r"].position = (
            self[idx_body].position + self[idx_body].axes_length[0] * 0.5 * ellbow_dir
        )

    def update_upperarm(self):
        idx_body = self.get_obstacle_id_from_name("upperarm_r")
        if self.margin_upperarms is None:
            # Assume displacement of reference point along z-axis
            self.margin_upperarms = (
                self[idx_body].axes_length[2]
                * 0.5
                / LA.norm(self[idx_body].get_reference_point(in_global_frame=True))
            )

        self[idx_body].position = 0.5 * (
            self.joints["shoulder_r"].position + self.joints["ellbow_r"].position
        )

        vect_upper = (
            self.joints["shoulder_r"].position - self.joints["ellbow_r"].position
        )
        self[idx_body].pose.orientation = get_rotation_between_vectors(
            [0.0, 0.0, 1.0], vect_upper
        )

        # Update size of upperarm to fit in between
        self[idx_body].axes_length[2] = self.margin_upperarms * 2 + LA.norm(vect_upper)
        reference_point = self[idx_body].get_reference_point(in_global_frame=True)
        reference_point[2] = LA.norm(vect_upper)
        self[idx_body].set_reference_point(reference_point, in_global_frame=False)

    def update_using_optitrack(self, transform_to_robot_frame: bool = True) -> None:
        if self.pose_updater is not None:
            self.new_object_poses = self.pose_updater.get_messages()

        else:
            self.new_object_poses = []
        self.indeces_measures = [oo.obs_id for oo in self.new_object_poses]
        self.indeces_optitrack_tree = self.optitrack_indeces

        self.update_robot()
        self.update_joints()

        self.update_body()
        # Forarm before upperarm, as we have a higher-collision likelihood (!)
        self.update_forearm()
        self.update_upperarm()

    def update_dynamic_obstacle(self, idx_obs: int, obs_measure: RigidBody):
        # Update position
        # self.position_filters[idx_obs].run_once(obs_measure.position)
        # self.orientation_filters[idx_obs].run_once(obs_measure.rotation)

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

    def update_orientation_based_on_position(
        self,
        idx_obs: int,
        # position_trust: float = 1.0
    ) -> None:
        idx_parent = list(self._graph.predecessors(idx_obs))[0]
        idx_local_ref = self._graph.nodes[idx_parent]["indeces_children"].index(idx_obs)
        local_reference_parent = self._graph.nodes[idx_parent]["references_children"][
            idx_local_ref
        ]
        reference_parent = self[idx_parent].pose.transform_position_from_relative(
            local_reference_parent
        )

        axes_direction = self[idx_obs].position - reference_parent
        # axes_direction = np.array([1.0, 0, -1.0])
        if not (axes_norm := np.linalg.norm(axes_direction)):
            # No information from position only
            return
        axes_direction = axes_direction / axes_norm

        rot_vec = np.cross([0.0, 0, 1.0], axes_direction)

        if rotvec_norm := np.linalg.norm(rot_vec):
            rot_vec = rot_vec / rotvec_norm
            theta = np.arcsin(rotvec_norm)
            quat = np.hstack((rot_vec * np.cos(theta / 2.0), [np.sin(theta / 2.0)]))

        else:
            quat = np.array([0, 0, 0, 1.0])

        # breakpoint()

        self[idx_obs].pose.orientation = Rotation.from_quat(quat)

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

        if self.robot is not None:
            self.robot.publish_robot_transform()

        if self.visualization_handler is not None:
            self.visualization_handler.update(self._obstacle_list)

    def get_gamma(self, position: Vector, in_global_frame: bool = True) -> bool:
        # in_global_frame is not used but kept for compatibility

        # Get minimum gamma
        gammas = np.zeros(self.n_components)

        for ii in range(self.n_components):
            gammas[ii] = self._obstacle_list[ii].get_gamma(
                position, in_global_frame=True
            )

        return np.min(gammas)
