import numpy as np
from manifpy import SO3, SO3Tangent
from numpy.typing import NDArray


class EKF:
    class State:
        a: NDArray
        q: SO3
        w: SO3Tangent
        theta: NDArray
        theta_dot: NDArray

    class Measurement:
        encoders: NDArray
        accelerations: NDArray
        gyroscope: NDArray

    def __init__(self):
        self.state = self.State()
