import numpy as np
import math
import pybullet as p
import random


# Calculate vertices accumutively


class Terrain:
    def __init__(self, client):
        """
        Make the terrain
        """

        self._client = client
        
        self.heightPerturbationRange = 0.2
        self.numHeightfieldRows = 100
        self.numHeightfieldColumns = 100
        heightfieldData = np.zeros((self.numHeightfieldColumns, self.numHeightfieldRows), dtype=float)

        for i in range(int(self.numHeightfieldColumns/2)):
            for j in range(int(self.numHeightfieldRows)):
                n1 = 0
                n2 = 0
                if j > 0:
                    n1 = heightfieldData[i, j-1]
                if i > 0:
                    n2 = heightfieldData[i-1, j]
                else:
                    n2 = n1
                noise = random.uniform(-self.heightPerturbationRange,
                                        self.heightPerturbationRange)
                heightfieldData[i, j] = (n1+n2)/2 + noise

        heightfieldData_inv = heightfieldData[::-1,:]
        heightfieldData_2 = np.concatenate((heightfieldData_inv, heightfieldData))

        col,row = heightfieldData_2.shape
        heightfieldData_2 = heightfieldData_2.reshape(-1)

        terrainShape = self._client.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, heightfieldData=heightfieldData_2, meshScale=[0.5,0.5,1],
                                                numHeightfieldRows=row, numHeightfieldColumns=col)
        terrain = self._client.createMultiBody(0, terrainShape)
        self._client.resetBasePositionAndOrientation(terrain, [0, 0, 0], [0, 0, 0, 1])

