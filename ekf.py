import numpy as np
from manifpy import SO3, SO3Tangent
from numpy.typing import NDArray


class EKF:
    # From (3)
    class State:
        a: NDArray  # World frame acceleration
        r: SO3  # World frame orientation
        w: SO3Tangent  # World frame angular velocity
        theta: NDArray
        theta_dot: NDArray

    class Measurement:
        encoders: NDArray
        accelerations: NDArray
        gyroscope: NDArray

    def __init__(self, command_mix: float) -> None:
        self.state = self.State()
        self.command_mix = command_mix

    def predict(self, u: NDArray, dt: float) -> None:
        """
        :param u:   Commanded joint velocities
        :param dt:  Time step
        """
        # From (5/6), but in lie form
        self.state.r = self.state.r + SO3Tangent.exp(self.state.w * dt)
        # From (8)
        self.state.theta = self.state.theta + self.state.theta_dot * dt
        # From (9)
        self.state.theta_dot = (1 - self.command_mix) * self.state.theta_dot + self.command_mix * u

    def update(self, m: Measurement) -> None:
        """
        :param m:   Measurement
        """
        pass
