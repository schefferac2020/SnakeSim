import numpy as np
import math
import pybullet as p
import random
import noise


# Calculate vertices accumutively


class Terrain:
    def __init__(self, client):
        """
        Make the terrain
        """

        self._client = client

        self.rows = 128
        self.cols = 128

        vertical_scale = 32

        heightmap = np.zeros((self.cols, self.rows), dtype=float)
        for x in range(self.cols):
            for y in range(self.rows):
                heightmap[x, y] = vertical_scale * noise.pnoise2(x / self.cols, y / self.rows,
                                                                 octaves=6, persistence=0.5, lacunarity=2.0, base=0)

        terrain_shape = self._client.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, heightfieldData=heightmap.reshape(-1),
                                                          heightfieldTextureScaling=self.rows,
                                                          numHeightfieldRows=self.rows, numHeightfieldColumns=self.cols)
        terrain = self._client.createMultiBody(0, terrain_shape)
        self._client.resetBasePositionAndOrientation(terrain, [0, 0, 0], [0, 0, 0, 1])
