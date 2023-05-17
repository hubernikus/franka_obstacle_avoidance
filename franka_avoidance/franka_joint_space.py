#!/usr/bin/env python3
from typing import Optional
import math

import numpy as np
from numpy import linalg
from scipy.spatial.transform import Rotation

from dynamic_obstacle_avoidance.obstacles import CuboidXd as Cuboid

# from nonlinear_avoidance.controller import RotationalAvoider
from nonlinear_avoidance.vector_rotation import VectorRotationXd


def fast_directional_addition(
    vector1: np.ndarray,
    vector2: np.ndarray,
    weight: float = 1.0,
    normalize_input: bool = True,
):
    """The weight is added on vector2"""
    if normalize_input:
        vector1 = vector1 / linalg.norm(vector2)
        vector2 = vector2 / linalg.norm(vector2)

    vector_out = (1 - weight) * vector1 + weight * vector2
    if not (vec_norm := linalg.norm(vector_out)):
        raise ValueError(
            "Vector are opposing each other - directional summing not possible!"
        )
    return vector_out / vec_norm


class FrankaJointSpace:
    """
    Controller which allows avoiding the join-limits q_min / q_max of the franka controller by explointing the null-space of the 7DOF robot.
    """

    # Joint Limits
    q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    q_delta = q_max - q_min
    q_mean = 0.5 * (q_min + q_max)
    boundary = Cuboid(center_position=q_mean, axes_length=q_delta, is_boundary=True)
    boundary.boundary_power_factor = 2.0

    # def __init__(self, weight_power: float = 2.0) -> None:
    #     self.weight_power =

    def __init__(self, weight_power: float = 2):
        self.weight_power = weight_power

    def get_gamma_weight(
        self,
        relative_position: np.ndarray,
        *,
        gamma_max: float = 1.5,
        gamma_min: float = 1.0,
    ) -> float:
        gamma = self.boundary.get_gamma(relative_position, in_obstacle_frame=True)
        print("gamma", gamma)
        if gamma > gamma_max:
            return 0
        elif gamma < gamma_min:
            return 1

        return (gamma_max - gamma) / (gamma_max - gamma_min)

    def get_rotation_weight(
        self,
        normal_direction: np.ndarray,
        initial_direction: np.ndarray,
        gamma_value: float,
        *,
        gamma_min: float = 1.0,
        gamma_max: float = 1.1,
        radius_min: float = np.pi * 3.0 / 4,
        radius_max: float = np.pi,
    ) -> float:
        """The weight is calculated based on normal and the desired direction"""

        if gamma_value >= gamma_max:
            return 0.0
        elif gamma_value <= gamma_min:
            return 1.0
        gamma_weight = (gamma_max - gamma_value) / (gamma_max - gamma_min)

        norm_radius = np.arccos(np.dot(normal_direction, initial_direction))
        if norm_radius <= radius_min:
            return gamma_weight
        elif norm_radius >= radius_max:
            return 0.0
        weight_power = (radius_max - radius_min) / (radius_max - norm_radius)

        return gamma_weight**weight_power

    def get_ideal_avoidance_direction(
        self,
        weight_rotation: float,
        weight_nullspace: float,
        velocity_null_space: np.ndarray,
        velocity_normal: np.ndarray,
    ) -> np.ndarray:
        weight = np.min((1.0 - weight_rotation), weight_nullspace)

        vector_rotation = VectorRotationXd.from_directions(
            velocity_normal, velocity_null_space
        )
        return vector_rotation.rotate(velocity_normal, rot_factor=weight)

    def get_rotated_velocity(
        self,
        weight_rotation: float,
        velocity_avoidance: np.ndarray,
        velocity_initial: np.ndarray,
    ) -> np.ndarray:
        vector_rotation = VectorRotationXd.from_directions(
            velocity_initial, velocity_avoidance
        )
        return vector_rotation.rotate(velocity_initial, rot_factor=weight_rotation)

    def get_limit_avoidance_velocity(
        self,
        joint_position: np.ndarray,
        joint_velocity: np.ndarray,
        jacobian: np.ndarray,
    ) -> np.ndarray:
        """Initial Null-Space pulling - BUT this is not continuous / leads to switching (!)."""
        # TODO: watch out - switching at the back -> use RotationalAvoider (!)
        if not (joint_speed := linalg.norm(joint_velocity)):
            return joint_velocity

        joint_velocity = joint_velocity / joint_speed
        relative_position = joint_position - self.q_mean

        gamma = self.boundary.get_gamma(relative_position, in_obstacle_frame=True)
        normal_direction = self.boundary.get_normal_direction(
            relative_position, in_obstacle_frame=True
        )
        rotation_weight = self.get_rotation_weight(
            normal_direction, joint_velocity, gamma
        )
        if rotation_weight <= 0:
            return joint_velocity * joint_speed

        nullspace_direction = self.get_nullspace_direction(jacobian)

        nullspace_weight = np.dot(nullspace_direction, normal_direction)

        if nullspace_weight < 0:
            nullspace_direction = -nullspace_direction
            nullspace_weight = -nullspace_weight

        normal_weight = min((1.0 - rotation_weight), nullspace_weight)

        optimal_avoidance_diection = fast_directional_addition(
            normal_direction, nullspace_direction, weight=normal_weight
        )

        rotated_velocity = fast_directional_addition(
            joint_velocity, optimal_avoidance_diection, weight=rotation_weight
        )

        # breakpoint()
        # Keep velocity magnitude
        return rotated_velocity * joint_speed

    @staticmethod
    def get_nullspace_direction(jacobian: np.ndarray):
        _, _, uv = linalg.svd(jacobian, compute_uv=True, full_matrices=True)
        return uv[-1, :]

    @classmethod
    def get_velocity_away_from_limits(
        cls, joint_position: np.ndarray, jacobian: np.ndarray
    ) -> np.ndarray:
        # PROBLEM: this can lead to jittering when in-between two cornerns...
        dist_qmin = cls.q_min - joint_position
        dist_qmax = cls.q_max - joint_position

        dist_q = np.vstack((dist_qmin, dist_qmax)) / (
            np.tile(cls.q_delta * 0.5, (2, 1))
        )

        idx_closest = np.argmin(np.abs(dist_q))
        idx_closest = np.unravel_index(idx_closest, dist_q.shape)

        weight = 1 - abs(dist_q[idx_closest])

        weight = weight**cls.weight_power

        if not weight:
            return np.zeros_like(joint_position)

        null_velocity = cls.find_null_space_velocity(jacobian)

        if (null_velocity[idx_closest[1]] > 0 and idx_closest[0] == 1) or (
            null_velocity[idx_closest[1]] < 0 and idx_closest[0] == 0
        ):
            null_velocity = -null_velocity
            # weight = -weight

        return null_velocity, weight
