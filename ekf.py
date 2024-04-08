import numpy as np
from manifpy import SO3, SO3Tangent
from numpy.typing import NDArray


class EKF:
    # From (3)
    class State:
        a: NDArray  # World frame acceleration
        R: SO3  # World frame orientation
        w: SO3Tangent  # World frame angular velocity
        theta: NDArray  # Joint angles
        theta_dot: NDArray  # Joint velocities

    class Measurement:
        encoders: NDArray
        accelerations: NDArray
        gyroscope: NDArray

    def __init__(self, num_joints: int, command_mix: float) -> None:
        self.N = num_joints
        self.state = self.State()
        self.P = np.eye(self.N) * 1e-6
        self.Q = np.eye(self.N) * 1e-6
        self.R = np.eye(self.N) * 1e-6

        self.g = np.array([0, 0, 9.81])
        self.command_mix = command_mix

    def predict(self, u: NDArray, dt: float) -> None:
        """
        :param u:   Commanded joint velocities
        :param dt:  Time step
        """
        # From (5/6), but in lie form
        self.state.R = self.state.R + self.state.w * dt
        # From (8)
        self.state.theta = self.state.theta + self.state.theta_dot * dt
        # From (9)
        self.state.theta_dot = (1 - self.command_mix) * self.state.theta_dot + self.command_mix * u

        F = ... # TODO: calculate jacobian of state dynamics
        self.P = F @ self.P @ F.T + self.Q

    def update(self, m: Measurement) -> None:
        """
        :param m:   Measurement
        """
        predicted_encoders = self.state.theta
        predicted_accelerations = self.acceleration_prediction()
        predicted_gyroscope = self.gyroscope_prediction()
        
        m_vec = np.concatenate([m.encoders, m.accelerations, m.gyroscope])
        predicted_vec = np.concatenate([predicted_encoders, predicted_accelerations, predicted_gyroscope])
        H = ... # TODO: calculate jacobian of measurement model

        innovation = m_vec - predicted_vec
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ innovation
        self.P = (np.eye(self.N) - K @ H) @ self.P

    def acceleration_prediction(self) -> NDArray:
        accel = np.zeros(3 * self.N)
        for i in range(self.N):

            # predicted acceleration due to gravity
            link_to_body = forward_kinematics(i, self.state.theta)
            world_to_link = self.state.R @ link_to_body # TODO: is this right? opposite of what is in the paper
            accel_g = world_to_link @ self.g
    
            # predicted acceleration due to internal motion
            accel_internal = ... # TODO
            
            # predicted acceleration from robot motion
            accel_robot = world_to_link @ self.state.a
            accel[3*i:3*(i+1)] = accel_g + accel_internal + accel_robot
        return accel
    
    def gyroscope_prediction(self) -> NDArray:
        gyro = np.zeros(3 * self.N)
        for i in range(self.N):
            link_to_body = forward_kinematics(i, self.state.theta)
            theta_prev = ... # TODO: get previous joint angles
            link_to_body_prev = forward_kinematics(i, theta_prev)
            ...
        return gyro