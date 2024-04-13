import noise
import numpy as np
import pybullet as p


class Terrain:
    def __init__(self, client):
        """
        Make the terrain
        """

        self._client = client

        rows, cols = 16, 16
        lateral_scale = 2
        vertical_scale = 8

        self.heightmap = np.zeros((cols, rows), dtype=float)
        for x in range(cols):
            for y in range(rows):
                # Perlin needs inputs between 0 and 1
                # We also need to scale as it outputs between -1 and 1
                self.heightmap[x, y] = vertical_scale * noise.pnoise2(
                    x / cols / lateral_scale, y / rows / lateral_scale,
                    octaves=6, persistence=0.5, lacunarity=2.0, base=0,
                )

        self.terrain_shape = self._client.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD, heightfieldData=self.heightmap.T.reshape(-1),
            heightfieldTextureScaling=rows - 1,
            numHeightfieldRows=rows, numHeightfieldColumns=cols,
        )
        self.terrain_multi_body = self._client.createMultiBody(0, self.terrain_shape)

        # Precompute normals
        self.normals = np.zeros((cols, rows, 3), dtype=float)
        for x in range(1, cols):
            for y in range(1, rows):
                # Shoot a ray from the sky directly above this cell and collect the hit normal
                # I figured this would be most similar to the snake giving back normal values from contact
                x_c = x - cols / 2
                y_c = y - rows / 2
                hit, = client.rayTest([x_c, y_c, 100], [x_c, y_c, -100])
                hit_normal = hit[4]
                self.normals[x, y] = hit_normal
                point = np.array([x_c, y_c, self.heightmap[x, y]])
                client.addUserDebugLine(point, point + hit_normal, [1, 0, 0])
