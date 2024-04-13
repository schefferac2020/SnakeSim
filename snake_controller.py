import numpy as np


class SnakeController:
    N: int

    def __init__(self, N: int):
        self.N = N

    def rolling_gait(self, t) -> np.ndarray:
        num_joints = self.N
        v = 3
        A = 0.3
        xi = v * t
        theta_odd = A * np.sin(xi)
        theta_even = A * np.sin(xi + np.pi / 2)
        thetas = np.zeros(num_joints)
        thetas[::2] = theta_even
        thetas[1::2] = theta_odd
        return thetas

    def general_gait(self, t, offset_even, offset_odd, amp_even, amp_odd, spatial_frequency, time_frequency, phase_shift) -> np.ndarray:
        frequencies = spatial_frequency * np.arange(self.N) + time_frequency * t
        angles = np.zeros(self.N)
        angles[::2] = offset_even + amp_even * np.sin(frequencies[::2])
        angles[1::2] = offset_odd + amp_odd * np.sin(frequencies[1::2] + phase_shift)
        return angles

    def general_gait2(self, t, offsets_even, offsets_odd, amp_even, amp_odd, spatial_frequency_even, spatial_frequency_odd, time_frequency_even, time_frequency_odd, phase_shift) -> np.ndarray:
        frequencies = np.arange(self.N, dtype=float)
        frequencies[::2] = spatial_frequency_even * frequencies[::2] + time_frequency_even * t
        frequencies[1::2] = spatial_frequency_odd * frequencies[1::2] + time_frequency_odd * t
        angles = np.zeros(self.N)
        angles[::2] = offsets_even + amp_even * np.sin(frequencies[::2])
        angles[1::2] = offsets_odd + amp_odd * np.sin(frequencies[1::2] + phase_shift)
        return angles

    def inchworm_gait(self, t, speed, turn) -> np.ndarray:
        # test = self.general_gait(t, turn, 0, 0, 0.3, 0.7, speed, 0)
        return self.general_gait2(t, turn, 0, 0, 0.3, 1, 0.7, 0, speed, 0)

    def inchworm_s_gait(self, t, speed, curvature) -> np.ndarray:
        offsets = np.zeros(self.N // 2)
        offsets[:self.N // 4] = curvature
        offsets[self.N // 4:] = -curvature
        return self.general_gait2(t, offsets, 0, 0, 0.3, 1, 0.7, 0, speed, 0)
