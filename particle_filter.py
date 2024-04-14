from dataclasses import dataclass, field

import numpy as np
from manifpy import SE3, SO3, SE3Tangent
from numpy.typing import NDArray

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

    def M(self, u: SE3Tangent) -> NDArray:
        pass

    def prediction(self, ekf_orientation: SO3, commanded_twist: SE3Tangent):
        M = self.M(commanded_twist)
        LQ = np.linalg.cholesky(M)
        for particle in self.particles:
            u_sample = commanded_twist + np.random.normal(0, 1, 6) @ LQ
            particle.head_link = particle.head_link + u_sample

    def correction(self, joint_contact_normals_in_world: NDArray):
        pass
