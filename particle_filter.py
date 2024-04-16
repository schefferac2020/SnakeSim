import copy
from dataclasses import dataclass, field

import numpy as np
from manifpy import SE3, SO3, SE3Tangent
from numpy.typing import NDArray
from typing import List, Tuple

from terrain import Terrain


class TerrainParticleFilter:
    @dataclass
    class Particle:
        head_link: SE3 = field(default_factory=lambda: SE3.Identity())
        weight: float = 0

    def __init__(self, _link_count: int, num_particles: int, terrain: Terrain):
        self.num_particles = num_particles
        self.terrain = terrain
        self.particles = [self.Particle(weight=1 / num_particles) for _ in range(num_particles)]

    def likelihood(self, particle: Particle, link: int, joint_contact_normals_in_world: NDArray) -> float:
        return 1

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

    def correction(self, joint_contact_normals_in_world: List[Tuple[int, NDArray]]):
        for link, normal in joint_contact_normals_in_world:
            for particle in self.particles:
                particle.weight *= self.likelihood(particle, link, normal)

        # Normalize weights to sum to one
        total_weight = sum(particle.weight for particle in self.particles)
        for particle in self.particles:
            particle.weight /= total_weight

        # N_eff = 1 / sum(particle.weight for particle in self.particles)
        # if N_eff < self.num_particles:
        #     self.resample()

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
        # Compute the weighted mean
        mean = np.zeros(3)
        for particle in self.particles:
            mean += particle.head_link.translation() * particle.weight
        return mean
