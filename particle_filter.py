import copy
from dataclasses import dataclass, field

import numpy as np
from manifpy import SE3, SO3, SE3Tangent
from numpy.typing import NDArray
from typing import List, Tuple

from utils import forward_kinematics

from terrain import Terrain

import pybullet as p

RESOLUTION = 0.5


class TerrainParticleFilter:
    @dataclass
    class Particle:
        head_link: SE3 = field(default_factory=lambda: SE3.Identity())
        weight: float = 0

    def __init__(self, _link_count: int, terrain: Terrain):
        self.terrain = terrain

        # Initialize the particles to be uniformly distributed across the terrain
        self.particles = []
        w, h = terrain.heightmap.shape
        for x in np.arange(-w / 2, w / 2, RESOLUTION):
            for y in np.arange(-h / 2, h / 2, RESOLUTION):
                particle = self.Particle()
                particle.head_link.coeffs()[:3] = x, y, terrain.heightmap[int(x + w / 2), int(y + h / 2)]
                self.particles.append(particle)
        self.num_particles = len(self.particles)
        for particle in self.particles:
            particle.weight = 1 / self.num_particles

        self.points_id = None

    def draw_particles(self):
        kwargs = {}
        if self.points_id is not None:
            kwargs['replaceItemUniqueId'] = self.points_id

        particle_positions = [particle.head_link.translation() for particle in self.particles]
        particle_colors = [[particle.weight, 0, 1 - particle.weight] for particle in self.particles]
        self.points_id = p.addUserDebugPoints(particle_positions, particle_colors,
                                              pointSize=6, lifeTime=0, **kwargs)

    def likelihood(self, particle: Particle,
                   link: int, joint_contact_normals_in_world: NDArray,
                   joint_angles: NDArray, head_to_virtual: SE3) -> float:
        particle_position = particle.head_link.translation()
        begin = particle_position + np.array([0, 0, 1])
        end = particle_position + np.array([0, 0, -1])
        hits = p.rayTest(begin, end)

        ground_hit = None
        for hit in hits:
            if hit[0] == 0:
                ground_hit = hit
                break

        if ground_hit is None:
            return 0

        ray_test_normal = hit[4]
        return np.dot(ray_test_normal, joint_contact_normals_in_world)

        # def M(self, u: SE3Tangent, alphas: NDArray) -> NDArray:

    #     pass

    def prediction(self, head_orientation_estimate: SO3, commanded_twist: SE3Tangent):
        if commanded_twist.squaredWeightedNorm() < 1e-6:
            return

        # TODO: Use M
        # M = self.M(commanded_twist, np.array([]))
        # LQ = np.linalg.cholesky(M)

        for particle in self.particles:
            # Set the particle's orientation, this should be from the EKF's output
            particle.head_link.coeffs()[3:] = head_orientation_estimate.quat()

            perturbation = SE3Tangent(np.random.normal(0, 1, 6) * 0.1 * commanded_twist.coeffs_copy() ** 2)
            sample_twist = commanded_twist + perturbation
            particle.head_link = particle.head_link + sample_twist

    def correction(self, joint_contact_normals_in_world: List[Tuple[int, NDArray]],
                   joint_angles: NDArray, head_to_virtual: SE3):
        for link, normal in joint_contact_normals_in_world:
            for particle in self.particles:
                particle.weight *= self.likelihood(particle, link, normal, joint_angles, head_to_virtual)

        # Normalize weights to sum to one
        total_weight = sum(particle.weight for particle in self.particles)
        for particle in self.particles:
            particle.weight /= total_weight

        N_eff = 1 / sum(particle.weight ** 2 for particle in self.particles)
        if N_eff < self.num_particles:
            self.resample()

    def resample(self):
        new_samples = []
        W = np.cumsum([particle.weight for particle in self.particles])
        r = np.random.rand() / self.num_particles
        count = 0
        for i in range(self.num_particles):
            u = r + i / self.num_particles
            while u > W[count]:
                count += 1
            new_particle = copy.deepcopy(self.particles[count])
            new_particle.weight = 1 / self.num_particles
            new_samples.append(new_particle)
        self.particles = new_samples

    def filter(self) -> NDArray:
        self.draw_particles()

        # Compute the weighted mean
        mean = np.zeros(3)
        for particle in self.particles:
            mean += particle.head_link.translation() * particle.weight
        return mean
