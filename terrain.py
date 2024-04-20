import noise
import numpy as np
import pybullet as p

from numpy.typing import NDArray


class Terrain:
    def grid_to_world(self, x, y) -> NDArray:
        t = np.array([x, y, self.heightmap[x, y]])
        t[:2] -= np.array(self.heightmap.shape) / 2
        return t

    def world_to_grid(self, x, y) -> NDArray:
        t = np.array([x, y])
        t += np.array(self.heightmap.shape) / 2
        return t.astype(int)

    def __init__(self, size=12, lateral_scale=2, vertical_scale=8) -> None:
        self.heightmap = np.zeros((size, size), dtype=float)
        for x in range(size):
            for y in range(size):
                # Perlin needs inputs between 0 and 1
                # We also need to scale as it outputs between -1 and 1
                self.heightmap[x, y] = vertical_scale * noise.pnoise2(
                    x / size / lateral_scale, y / size / lateral_scale,
                    octaves=6, persistence=0.5, lacunarity=2.0, base=0,
                )

        self.terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD, heightfieldData=self.heightmap.T.reshape(-1),
            heightfieldTextureScaling=size - 1,
            numHeightfieldRows=size, numHeightfieldColumns=size,
        )
        self.terrain_multi_body = p.createMultiBody(0, self.terrain_shape)

        # # Precompute normals
        # self.normals = np.zeros((cols, rows, 3), dtype=float)
        # for x in range(1, cols):
        #     for y in range(1, rows):
        #         # Shoot a ray from the sky directly above this cell and collect the hit normal
        #         # I figured this would be most similar to the snake giving back normal values from contact
        #         x_c = x - cols / 2
        #         y_c = y - rows / 2
        #         hit, = p.rayTest([x_c, y_c, 100], [x_c, y_c, -100])
        #         hit_normal = hit[4]
        #         self.normals[x, y] = hit_normal
        #         point = np.array([x_c, y_c, self.heightmap[x, y]])
        #         p.addUserDebugLine(point, point + hit_normal, [1, 0, 0])
