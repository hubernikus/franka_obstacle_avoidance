"""
Container which smoothenes position (and rotation) of incoming obstacles.
"""
import warnings

import numpy as np
from numpy import linalg as LA
from scipy.spatial.transform import Rotation

from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter


def get_angular_velocity_from_quaterions(
    q1: Rotation, q2: Rotation, dt: float
) -> np.ndarray:
    """Returns the angular velocity required to reach quaternion 'q2'
    from quaternion 'q1' within the timestep 'dt'."""
    delta_q = q2.apply(q1.inv)
    delta_q = delta_q / LA.norm(delta_q)

    delta_q_norm = LA.norm(delta_q[1:])
    delta_q_angle = 2 * np.arctan2(delta_q_norm, delta_q[0])
    return delta_q[1:] * (delta_q_angle / dt)


class OrientationFilter:
    """Orientation filter easy to use with optitrack."""

    def __init__(self, update_frequency: float = 100.0):
        # Measure Quaternion - Estimate Quaternion and Position
        # self._kf = KalmanFilter(dim_x=7, dim_z=4)
        self._kf = KalmanFilter(dim_x=7, dim_z=7)

        self.dim_quat = 4
        self.dim_vel = 3

        self.dt = 1.0 / update_frequency

        self.x = np.zeros(self._kf.dim_x)

        # State transition matrix (dummy matrix for now)
        self._kf.F = np.eye(self._kf.dim_x)

        # Measurement function (measures position only)
        # self._kf.H = np.vstack((np.eye(4), np.zeros((3, 4))))
        self._kf.F = np.eye(self._kf.dim_x)

        # Covariance matrix
        self._kf.P = np.eye(self._kf.dim_x)

        # Measurement noise
        self._kf.R = np.eye(self._kf.dim_x) * 0.01

        # Process noise
        self._kf.Q = Q_discrete_white_noise(dim=self.dim_x, dt=self.dt, var=0.01)

    def compute_once(self, quaternion_measurement: np.ndarray):
        velocity_estimate = get_angular_velocity_from_quaterions(
            self.orientation,
            Rotation.from_quat(quaternion_measurement),
            self.dt,
        )

        self.update_state_transition()
        self._kf.predict()
        self._kf.update(np.hstack((quaternion_measurement, velocity_estimate)))

    def update_state_transition(self):
        self._kf.F = np.eye(self.dim_x)

        # Quaternion / Get rotation matrix
        wx = self.velocity[0]
        wy = self.velocity[1]
        wz = self.velocity[2]

        Omega_t = np.zeros(4, 4)
        Omega_t[0, :] = [0, -wx, -wy, -wz]
        Omega_t[1, :] = [wx, 0, wz, -wy]
        Omega_t[2, :] = [wy, -wz, 0, wx]
        Omega_t[3, :] = [wz, wy, -wx, 0]

        self._kf.F[:4, :][:, 4:] = 0.5 * self.dt * Omega_t

    @property
    def quaternion(self) -> np.ndarray:
        return self._kf.x[:4]

    @property
    def orientation(self) -> Rotation:
        return Rotation.from_quat(self._kf.x[:4])

    def reset_quaternion(self, value) -> None:
        self._kf.x[:4] = value

    @property
    def velocity(self) -> np.ndarray:
        return self._kf.x[4:]

    def reset_velocity(self, value) -> None:
        self._kf.x[4:] = value


class PositionFilter:
    """Implementation of kalman filter for position
    -> x-y-z could be separated / simpler filter as they are independant."""

    def __init__(self, update_frequency: float = 100.0):
        self.dimension = 3

        # Measure Position - Estimate Velocity
        # self._kf = KalmanFilter(dim_x=self.dimension * 2, dim_z=self.dimension)
        self._kf = KalmanFilter(dim_x=self.dimension * 2, dim_z=self.dimension * 2)

        self.dt = 1.0 / update_frequency

        self._kf.x = np.zeros(self._kf.dim_x)

        # State transition matrix
        self._kf.F = np.eye(self._kf.dim_x)
        self._kf.F[:, : self.dimension][:, self.dimension :] = np.eye(self.dimension)

        # Measurement function (measures position only)
        # self._kf.H = np.hstack((np.eye(self.dimension), np.zeros(self.dimension)))
        self._kf.H = np.eye(self._kf.dim_x)

        # Covariance matrix
        self._kf.P = np.eye(self._kf.dim_x)

        # Measurement noise
        self._kf.R = np.eye(self._kf.dim_x) * 0.01

        # Process noise
        # self._kf.Q = Q_discrete_white_noise(dim=self.dimension, dt=self.dt, var=0.01)
        self._kf.Q = Q_discrete_white_noise(dim=self._kf.dim_x, dt=self.dt, var=0.01)

    def run_once(self, position_measurement: np.ndarray) -> None:
        velocity_estimate = (position_measurement - self.position) / self.dt

        self._kf.predict()
        self._kf.update(np.hstack((position_measurement, velocity_estimate)))

    def reset_position(self, value: np.ndarray):
        self._kf.x[: self.dimension] = value

    @property
    def position(self) -> np.ndarray:
        return self._kf.x[: self.dimension]

    def reset_velocity(self, value: np.ndarray) -> None:
        self._kf.x[self.dimension :] = value

    @property
    def velocity(self) -> np.ndarray:
        return self._kf.x[self.dimension :]


class UnfinishedFilter:
    # class UnfinishedFilter(ExtendedKalmanFilter):
    def __init__(self, update_rate: float):
        self.dim_x = 7 + 6 + 6
        # Position and Orientation
        self.dim_z = 7

        self.delta_time = 1 / update_rate

    def get_jacobian(self, pos, dt):
        J = np.eye(self.dim_x)

        # Position
        J[0:3, :][:, 3:6] = dt

        # Linear Velocity
        J[3:6, :][:, 6:9] = dt

        # Quaternion / Get rotation matrix
        q_omega_matrix = np.zeros((4, 4))
        # Column 0
        q_omega_matrix[1, 0] = self.angular_velocity[0]
        q_omega_matrix[2, 0] = self.angular_velocity[1]
        q_omega_matrix[3, 0] = self.angular_velocity[2]
        # Column 1
        q_omega_matrix[0, 1] = -self.angular_velocity[0]
        q_omega_matrix[2, 1] = -self.angular_velocity[2]
        q_omega_matrix[3, 1] = self.angular_velocity[1]
        # Column 2
        q_omega_matrix[0, 2] = -self.angular_velocity[1]
        q_omega_matrix[1, 2] = self.angular_velocity[2]
        q_omega_matrix[3, 2] = -self.angular_velocity[0]
        # Column 3
        q_omega_matrix[0, 3] = -self.angular_velocity[2]
        q_omega_matrix[1, 3] = -self.angular_velocity[1]
        q_omega_matrix[2, 3] = self.angular_velocity[0]
        # Assign
        J[9:13, :][:, 13:16] = 0.5 * q_omega_matrix * dt

        # Angular Velocity
        J[13:16, :][:, 16:19] = dt

    def _normalize_quaternion(self) -> None:
        quat_norm = LA.norm(self.quaternion)
        if quat_norm:
            self.quaternion = self.quaternion / quat_norm
        else:
            qq = np.zeros(4)
            qq[0] = 1
            self.quaternion = qq
            return

    @property
    def position(self):
        """Returns the orientation"""
        return self.x[0:3]

    @property
    def linear_velocity(self):
        return self.x[3:6]

    @property
    def linear_acceleration(self):
        return self.x[6:9]

    def get_orientation(self) -> Rotation:
        """Returns the orientation as a quaternion"""
        return Rotation.from_quat(self.x[9:13])

    @property
    def quaternion(self):
        """Returns the orientation as a quaternion"""
        return self.x[9:13]

    @quaternion.setter
    def quaternion(self, value: np.ndarray):
        """Returns the orientation as a quaternion"""
        self.x[9:13] = value

    @property
    def angular_velocity(self):
        return self.x[13:16]

    @property
    def angular_acceleration(self) -> np.ndarray:
        return self.x[16:19]


def test_position_filter():
    position_measurements = [
        [0, 1, 0],
        [0.1, 1.1, -0.1],
        [0.2, 1.2, -0.2],
        [0.3, 1.3, -0.3],
        [0.4, 1.4, -0.4],
        [0.6, 1.4, -0.4],
    ]

    for ii, pos in enumerate(position_measurement):
        pass


if (__name__) == "__main__":
    main()
