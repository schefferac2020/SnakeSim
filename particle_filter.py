import numpy as np
from manifpy import SE3d, SO3d
from numpy.typing import NDArray

import Terrain


class TerrainParticleFilter:
    class Particle:
        joints_in_world: list[SE3d]

    def __init__(self, num_particles: int, terrain: Terrain):
        self.num_particles = num_particles
        self.terrain = terrain
        self.particles = [self.Particle() for _ in range(num_particles)]

    def likelihood(self, particle: Particle, joint_contact_normals_in_world: NDArray):
        score = 0
        for joint_in_world, joint_contact_normal_in_world in zip(particle.joints_in_world, joint_contact_normals_in_world):
            terrain_normal_in_world = ...
            score += np.dot(joint_contact_normal_in_world, terrain_normal_in_world)
        return score

    def prediction(self, joint_orientations_in_world: list[SO3d], joint_angular_velocities: NDArray):
        for particle in self.particles:
            pass

    def correction(self, joint_contact_normals_in_world: NDArray):
        pass
