import numpy as np
from manifpy import SE3, SO3, SE3Tangent
from numpy.typing import NDArray

import terrain


class TerrainParticleFilter:
    class Particle:
        pose: SE3

    def __init__(self, num_particles: int, terrain: Terrain):
        self.num_particles = num_particles
        self.terrain = terrain
        self.particles = [self.Particle() for _ in range(num_particles)]

    def likelihood(self, particle: Particle, joint_contact_normals_in_world: NDArray):
        pass

    def prediction(self, ekf_orientation: SO3, commanded_twist: SE3Tangent):
        pass

    def correction(self, joint_contact_normals_in_world: NDArray):
        pass
