"""
Human with flexible limbs lengths (depending on marker-update)
"""
# Author: Lukas Huber
# Github: hubernikus
# Created: 2023-05-17

from __future__ import annotations  # Self typing

from dataclasses import dataclass, field
from typing import Optional
import warnings

import numpy as np
import numpy.typing as npt
from numpy import linalg as LA

from scipy.spatial.transform import Rotation

import networkx as nx

from vartools.states import ObjectPose
from vartools.state_filters import PositionFilter, SimpleOrientationFilter
from vartools.linalg import get_rotation_between_vectors

from dynamic_obstacle_avoidance.containers import ObstacleContainer
from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid
from dynamic_obstacle_avoidance.obstacles import EllipseWithAxes as Ellipse

from nonlinear_avoidance.rigid_body import RigidBody
from nonlinear_avoidance.multi_obstacle_avoider import MultiObstacleAvoider
from nonlinear_avoidance.dynamics.circular_dynamics import SimpleCircularDynamics
from nonlinear_avoidance.dynamics.projected_rotation_dynamics import (
    ProjectedRotationDynamics,
)


@dataclass
class Joint:
    # If needed - a filter could be added here...
    id_optitrack: int
    position: np.ndarray
    orientation: Optional[Rotation] = None
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
        id_forearm_r: Optional[int] = None,
        id_body: Optional[int] = None,
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

        self.optitracks: dict[str, Joint] = {}
        self.optitracks["ellbow_r"] = Joint(id_ellbow_r, np.zeros(self.dimension))
        self.optitracks["wrist_r"] = Joint(id_wrist_r, np.zeros(self.dimension))
        self.optitracks["shoulder_r"] = Joint(id_shoulder_r, np.zeros(self.dimension))
        self.optitracks["shoulder_l"] = Joint(id_shoulder_l, np.zeros(self.dimension))
        self.optitracks["shoulder_l"] = Joint(id_shoulder_l, np.zeros(self.dimension))
        self.optitracks["forearm_r"] = Joint(id_forearm_r, np.zeros(self.dimension))
        self.optitracks["body"] = Joint(id_body, np.zeros(self.dimension))

        self.margin_upperarms: Optional[float] = None
        self.ellbow_joint_dist: Optional[float] = None

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

        # print(f"Name : {name} --- number {len(self._obstacle_list)}")
        # breakpoint()

    @property
    def optitrack_indeces(self):
        indeces_tree_opti = [-1] * self.n_components
        for ii in range(self.n_components):
            if self._graph.nodes[ii]["update_id"] is not None:
                indeces_tree_opti[ii] = self._graph.nodes[ii]["update_id"]

        return indeces_tree_opti

    def update_optitracks(self) -> None:
        # print(self.indeces_measures)
        for kk in self.optitracks.keys():
            try:
                idx_joint = self.indeces_measures.index(
                    self.optitracks[kk].id_optitrack
                )

            except ValueError:
                # Element n ot in list
                continue

            self.optitracks[
                kk
            ].position = self.robot.pose.transform_position_to_relative(
                self.new_object_poses[idx_joint].position
            )

            self.optitracks[
                kk
            ].orientation = self.robot.pose.transform_orientation_to_relative(
                self.new_object_poses[idx_joint].rotation
            )
            # if kk == "forearm_r" or kk == "body":
            #     breakpoint()

    def update_robot(self):
        if self.robot is None:
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

    def update_body(self, use_joints: bool = False):
        """Body is set to be in between shoulders."""
        idx_body = self.get_obstacle_id_from_name("body")

        if self.ellbow_joint_dist is None:
            self.ellbow_joint_dist = LA.norm(
                self[idx_body].get_reference_point(in_global_frame=False)
            )

        if use_joints:
            # Use shoulde rlinks - or directly a body-placement
            self[idx_body].position = 0.5 * (
                self.optitracks["shoulder_l"].position
                + self.optitracks["shoulder_r"].position
            )

            shoulder_vec = (
                self.optitracks["shoulder_r"].position
                - self.optitracks["shoulder_l"].position
            )

            # shoulder_vec = [1.0, 0, 0]
            self[idx_body].pose.orientation = get_rotation_between_vectors(
                [0, 1.0, 0],
                shoulder_vec,
            )
        else:
            self[idx_body].position = self.optitracks["body"].position
            if self.optitracks["body"].orientation is None:
                self[idx_body].orientation = Rotation.from_rotvec([0, 0, 0])

            else:
                self[idx_body].orientation = self.optitracks["body"].orientation

        # Limit movement and orietnation
        self[idx_body].position[2] = 0.4

        zyx_rot = self[idx_body].pose.orientation.as_euler("zyx")
        self[idx_body].pose.orientation = Rotation.from_euler("z", zyx_rot[0])
        # self[idx_body].pose.orientation = Rotation.from_euler("z", 0)
        # breakpoint()

        # The optitracks now have to be updated - based on the position
        self.optitracks["shoulder_l"].position = self.get_joint_position_to_parent(
            "upperarm_l"
        )
        self.optitracks["shoulder_r"].position = self.get_joint_position_to_parent(
            "upperarm_r"
        )

    def update_forearm(self, use_joints: bool = False):
        idx_body = self.get_obstacle_id_from_name("forearm_r")

        if use_joints:
            self[idx_body].position = 0.5 * (
                self.optitracks["wrist_r"].position
                + self.optitracks["ellbow_r"].position
            )

            ellbow_dir = (
                self.optitracks["wrist_r"].position
                - self.optitracks["ellbow_r"].position
            )
            self[idx_body].pose.orientation = get_rotation_between_vectors(
                [0.0, 0.0, 1.0],
                ellbow_dir,
            )

            # Update optitracks to be in-between
            if ellbow_norm := np.linalg.norm(ellbow_dir):
                ellbow_dir = ellbow_dir / ellbow_norm
            else:
                ellbow_dir = np.array([0, 0, 1.0])
        else:
            self[idx_body].position = self.optitracks["forearm_r"].position
            if self.optitracks["forearm_r"].orientation is None:
                self[idx_body].orientation = Rotation.from_rotvec([0, 0, 0])
            else:
                self[idx_body].orientation = self.optitracks["forearm_r"].orientation

            ellbow_dir = self[idx_body].orientation.apply([0, 0, 1.0])

        self.optitracks["ellbow_r"].position = (
            self[idx_body].position - self.ellbow_joint_dist * ellbow_dir
        )
        self.optitracks["wrist_r"].position = (
            self[idx_body].position + self.ellbow_joint_dist * ellbow_dir
        )

    def update_upperarm(self):
        idx_body = self.get_obstacle_id_from_name("upperarm_r")
        if self.margin_upperarms is None:
            # Assume displacement of reference point along z-axis
            self.margin_upperarms = self[idx_body].axes_length[2] * 0.5 - LA.norm(
                self[idx_body].get_reference_point(in_global_frame=True)
            )

        self[idx_body].position = 0.5 * (
            self.optitracks["shoulder_r"].position
            + self.optitracks["ellbow_r"].position
        )

        vect_upper = (
            self.optitracks["ellbow_r"].position
            - self.optitracks["shoulder_r"].position
        )
        self[idx_body].pose.orientation = get_rotation_between_vectors(
            [0.0, 0.0, -1.0],
            vect_upper,
        )

        # Update size of upperarm to fit in between
        self[idx_body].axes_length[2] = self.margin_upperarms * 2 + LA.norm(vect_upper)

        # Since we're changing the shape - we empty the cache
        # maybe the cache could be reset partially.
        self.visualization_handler.empty_obstacle_cache()

        # reference_point = self[idx_body].get_reference_point(in_global_frame=True)
        # # reference_point[2] = self[idx_body].center_position + LA.norm(vect_upper)
        self[idx_body].set_reference_point(
            self.optitracks["shoulder_r"].position, in_global_frame=True
        )

    def update_using_optitrack(self, transform_to_robot_frame: bool = True) -> None:
        if self.pose_updater is not None:
            self.new_object_poses = self.pose_updater.get_messages()

        else:
            self.new_object_poses = []
        self.indeces_measures = [oo.obs_id for oo in self.new_object_poses]
        # self.indeces_optitrack_tree = self.optitrack_indeces

        self.update_robot()
        self.update_optitracks()

        self.update_body()
        # Forarm before upperarm, as we have a higher-collision likelihood (!)
        # and the upperarm length gets updated depending on the forear position
        self.update_forearm()
        self.update_upperarm()

        # Arm left - make just hang down - set assembe neck and head
        for limb_name in ["upperarm_l", "forearm_l", "neck", "head"]:
            idx_obs = self.get_obstacle_id_from_name(limb_name)
            self.align_position_with_parent(idx_obs)

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
