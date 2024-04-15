from dataclasses import dataclass, field

import numpy as np
from manifpy import SE3, SO3, SE3Tangent
from numpy.typing import NDArray, ArrayLike

from terrain import Terrain


class TerrainParticleFilter:
    @dataclass
    class Particle:
        head_link: SE3 = field(default_factory=lambda: SE3.Identity())

    def __init__(self, _link_count: int, num_particles: int, terrain: Terrain):
        self.num_particles = num_particles
        self.terrain = terrain
        self.particles = [self.Particle() for _ in range(num_particles)]

    def likelihood(self, particle: Particle, joint_contact_normals_in_world: NDArray):
        pass

    def M(self, u: SE3Tangent, alphas: ArrayLike) -> NDArray:
        # a0, a1, a2, a3, a4, a5 = alphas
        # p1, p2, p3, t1, t2, t3 = u.coeffs()
        # return np.array([[a0 * p1 ** 2 + a1 * p2 ** 2, 0, 0],
        #                  [0, a2 * p1 ** 2 + a3 * p2 ** 2, 0],
        #                  [0, 0, a4 * p1 ** 2 + a5 * p2 ** 2]])
        return np.eye(6) * 0.1

    def prediction(self, ekf_orientation: SO3, commanded_twist: SE3Tangent):
        if commanded_twist.squaredWeightedNorm() < 1e-6:
            return

        M = self.M(commanded_twist, None)
        LQ = np.linalg.cholesky(M)

        for particle in self.particles:
            u_sample = commanded_twist + SE3Tangent(np.random.normal(0, 1, 6) @ LQ)
            particle.head_link = particle.head_link + u_sample


def correction(self, joint_contact_normals_in_world: NDArray):
    pass
