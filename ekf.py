import numpy as np
from manifpy import SO3, SO3Tangent
from numpy.typing import NDArray


class EKF:
    class State:
        a: NDArray  # World frame acceleration
        q: SO3  # World frame orientation
        w: SO3Tangent
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
        self.state.q = self.state.q + SO3Tangent.exp(self.state.w * dt)
        self.state.theta = self.state.theta + self.state.theta_dot * dt
        self.state.theta_dot = (1 - self.command_mix) * self.state.theta_dot + self.command_mix * u
