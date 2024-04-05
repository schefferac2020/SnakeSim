import numpy as np

class SnakeController:
    N: int

    def __init__(self, N: int):
        self.N = N
    
    def rolling_gait(self, t) -> np.ndarray:
        num_joints = self.N
        v = 3
        A = 0.3
        xi = v*t
        theta_odd = A*np.sin(xi)
        theta_even = A*np.sin(xi + np.pi/2)
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

    def inchworm_gait(self, t, speed, turn) -> np.ndarray:
        return self.general_gait(t, turn, 0, 0, 0.3, 0.7, speed, 0)
