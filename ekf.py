import numpy as np
from manifpy import SO3, SO3Tangent, SE3, SE3Tangent
from numpy.typing import NDArray
from utils import forward_kinematics


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

    def __init__(self, num_joints: int, link_length: float, command_mix: float) -> None:
        self.N = num_joints
        self.l = link_length
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
            link_to_head = self.forward_kinematics(i, self.state.theta)
            head_to_body = ... # virtual chassis frame transform
            world_to_link = self.state.R @ head_to_body @ link_to_head # TODO: is this right? opposite of what is in the paper
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
            link_to_head = self.forward_kinematics(i, self.state.theta)
            head_to_body = ... # virtual chassis frame transform
            link_to_body = head_to_body @ link_to_head
            dt = ... # TODO: get time step
            theta_prev = ... # TODO: get previous joint angles
            link_to_head_prev = self.forward_kinematics(i, theta_prev)
            head_to_body_prev = ... # virtual chassis frame transform
            link_to_body_prev = head_to_body_prev @ link_to_head_prev
            delta_R = link_to_body @ link_to_body_prev.T / dt

            # extract angular velocity from skew components
            w_z = delta_R[1, 0]
            w_y = delta_R[0, 2]
            w_x = delta_R[2, 1]
            gyro_internal = np.array([w_x, w_y, w_z])

            gyro[3*i:3*(i+1)] = gyro_internal + link_to_body.T @ self.state.w
        return gyro
    
    def forward_kinematics(self, i: int, theta: NDArray) -> SE3:
        return forward_kinematics(i, self.l, theta)
