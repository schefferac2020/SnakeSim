import copy
from dataclasses import dataclass, field

import numpy as np
from manifpy import SE3, SO3, SE3Tangent
from numpy.typing import NDArray
from typing import List, Tuple, Dict

from utils import forward_kinematics

from terrain import Terrain

import pybullet as p

RESOLUTION = 0.5


class TerrainParticleFilter:
    @dataclass
    class Particle:
        vc_in_map: SE3 = field(default_factory=lambda: SE3.Identity())
        weight: float = 0

    def __init__(self, _link_count: int, terrain: Terrain) -> None:
        self.terrain = terrain

        # Initialize the particles to be uniformly distributed across the terrain
        self.particles = []
        w, h = terrain.heightmap.shape
        for x in np.arange(-(w - 2) / 2, (w - 1) / 2, RESOLUTION):
            for y in np.arange(-(h - 2) / 2, (h - 1) / 2, RESOLUTION):
                particle = self.Particle()
                particle.vc_in_map.coeffs()[:3] = x, y, terrain.heightmap[int(x + w / 2), int(y + h / 2)]
                self.particles.append(particle)
        self.num_particles = len(self.particles)
        for particle in self.particles:
            particle.weight = 1 / self.num_particles

        self.points_id = None

    def draw_particles(self) -> None:
        kwargs = {}
        if self.points_id is not None:
            kwargs['replaceItemUniqueId'] = self.points_id

        particle_positions = [particle.vc_in_map.translation() for particle in self.particles]
        maximal_weight = max(particle.weight for particle in self.particles)
        if maximal_weight == 0:
            maximal_weight = 1
        particle_colors = [[1 - particle.weight / maximal_weight, 0, particle.weight / maximal_weight] for particle in self.particles]
        self.points_id = p.addUserDebugPoints(particle_positions, particle_colors,
                                              pointSize=6, lifeTime=0, **kwargs)

    def compute_weights(self, joint_contact_normals_in_world: Dict[int, NDArray],
                        vc_to_head: SE3, joint_angles: NDArray, link_length: float) -> None:

        # Each particle represents a potential virtual chassis in the map frame
        # Each particle has joints associated with it, raycast for each one
        # Score each particle based on how well the raycasted normals align with the contact normals

        ray_cast_beings = []
        ray_cast_ends = []
        particles = []
        normal_from_sensors = []
        for particle in self.particles:
            for link, normal_from_sensor in joint_contact_normals_in_world.items():
                joint_to_head = forward_kinematics(link, link_length, joint_angles)
                joint_in_map = particle.vc_in_map * vc_to_head.inverse() * joint_to_head

                begin = joint_in_map.translation() + np.array([0, 0, 1])
                end = joint_in_map.translation() + np.array([0, 0, -1])

                ray_cast_beings.append(begin)
                ray_cast_ends.append(end)
                particles.append(particle)
                normal_from_sensors.append(normal_from_sensor)

        # Perform in batch for performance reasons despite poor readability
        results = p.rayTestBatch(ray_cast_beings, ray_cast_ends)
        for particle, hit, normal_from_sensor in zip(particles, results, normal_from_sensors):
            hit_id, _, _, _, normal_from_particle, *_ = hit
            if hit_id == 0:
                particle.weight += np.dot(normal_from_particle, normal_from_sensor)

        # def M(self, u: SE3Tangent, alphas: NDArray) -> NDArray:

    #     pass

    def prediction(self, vc_in_map: SO3, commanded_twist: SE3Tangent):
        for particle in self.particles:
            particle.vc_in_map.coeffs()[3:] = vc_in_map.coeffs_copy()

        if commanded_twist.squaredWeightedNorm() < 1e-6:
            return

        # TODO: Use M
        # M = self.M(commanded_twist, np.array([]))
        # LQ = np.linalg.cholesky(M)

        for particle in self.particles:
            ...
            # perturbation = SE3Tangent(np.random.normal(0, 1, 6) * 0.1 * commanded_twist.coeffs_copy() ** 2)
            # sample_twist = commanded_twist + perturbation
            # particle.pose = particle.pose + sample_twist

    def correction(self, joint_contact_normals_in_world: Dict[int, NDArray],
                   vc_to_head: SE3, joint_angles: NDArray, link_length: float) -> None:

        self.compute_weights(joint_contact_normals_in_world, vc_to_head, joint_angles, link_length)

        # Normalize weights to sum to one
        total_weight = sum(particle.weight for particle in self.particles)
        if total_weight == 0:
            # Gained no information from the contact normals
            return

        for particle in self.particles:
            particle.weight /= total_weight

        # N_eff = 1 / sum(particle.weight ** 2 for particle in self.particles)
        # if N_eff < self.num_particles:
        #     self.resample()

    def resample(self) -> None:
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
            mean += particle.vc_in_map.translation() * particle.weight
        return mean
