from dataclasses import dataclass, field
from typing import Dict

from terrain import Terrain
from utils import *

RESOLUTION = 2.0


class TerrainParticleFilter:
    @dataclass
    class Particle:
        vc_in_map: SE3 = field(default_factory=lambda: SE3.Identity())
        weight: float = 0

    def __init__(self, terrain: Terrain) -> None:
        self.terrain = terrain

        # Initialize the particles to be uniformly distributed across the terrain
        self.particles = []
        w, h = terrain.heightmap.shape
        for x in np.arange(-w / 2, w / 2, RESOLUTION):
            for y in np.arange(-h / 2, h / 2, RESOLUTION):
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
        particle_weights = normalize_array(np.array([particle.weight for particle in self.particles]))
        particle_colors = [[1 - weight, 0, weight] for weight in particle_weights]
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
        for link, normal_from_sensor in joint_contact_normals_in_world.items():
            joint_to_head = forward_kinematics(link, link_length, joint_angles)

            for particle in self.particles:
                joint_in_map = particle.vc_in_map * vc_to_head.inverse() * joint_to_head

                # Start a little above the joint and shoot a ray downward
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
            if hit_id == -1:
                particle.weight *= 0.3
            elif hit_id == 0:
                alignment = np.dot(normal_from_particle, normal_from_sensor)
                particle.weight *= (alignment + 1) / 2

    def prediction(self, vc_in_map: SO3, commanded_twist: SE3Tangent):
        # Affix the orientation of each particle
        for particle in self.particles:
            particle.vc_in_map.coeffs()[3:] = vc_in_map.coeffs_copy()

        for particle in self.particles:
            # Random walk model proportional to commanded twist magnitude
            # Clip so that all directions get the chance to be considered
            # Otherwise they clump into a shape and lose variety
            proportion = np.clip(0.01 * commanded_twist.coeffs() ** 2, 0.05, 1)
            particle.vc_in_map += SE3Tangent(np.random.normal(0, 1, 6) * proportion)

            # Snap the particles back down to the terrain
            grid_x, grid_y = self.terrain.world_to_grid(*particle.vc_in_map.translation()[:2])
            if 0 <= grid_x < self.terrain.heightmap.shape[0] and 0 <= grid_y < self.terrain.heightmap.shape[1]:
                particle.vc_in_map.coeffs()[2] = self.terrain.heightmap[grid_x, grid_y]

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

        # Draw before re-sampling to avoid them all being the same colo
        self.draw_particles()

        N_eff = 1 / sum(particle.weight ** 2 for particle in self.particles)
        if N_eff < self.num_particles:
            self.resample()

    def resample(self) -> None:
        new_samples = []
        W = np.cumsum([particle.weight for particle in self.particles])
        r = np.random.rand() / self.num_particles
        count = 0
        for i in range(self.num_particles):
            u = r + i / self.num_particles
            while u > W[count]:
                count += 1

            new_samples.append(self.Particle(
                vc_in_map=SE3(self.particles[count].vc_in_map.coeffs_copy()),
                weight=1 / self.num_particles,
            ))
        self.particles = new_samples

    def filter(self) -> NDArray:
        # Compute the weighted average of the particles
        # Only for position since they all have the same orientation
        average_position = np.zeros(3)
        for particle in self.particles:
            average_position += particle.vc_in_map.translation() * particle.weight
        return SE3(average_position, particle.vc_in_map.quat())
