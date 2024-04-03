
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc

from SnakeRobot import SnakeRobot
from Terrain import Terrain

import time

# Setup pybullet client
client = bc.BulletClient(connection_mode=p.GUI)
client.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
client.setGravity(0, 0, -10)
client.setRealTimeSimulation(1)


# Make the terrain
terrain = Terrain(client)
# planeId = client.loadURDF("plane.urdf")

# Make the snake
N = 16 # links other than the red head
snake = SnakeRobot(N, client, [0, 0, 5], [0, 0, 0, 1])

# Make the rest of the world
# startPos = [0,0,1]
# startOrientation = client.getQuaternionFromEuler([0,0,0])
#set the center of mass frame (loadURDF sets base link frame) startPos/OrnclientresetBasePositionAndOrientation(boxId, startPos, startOrientation)



desired_joint_ang = 0.2 * np.ones(N)

# Simulate
# TODO: use proper pybullet timing
dt = 1. / 240.
t_sim = 0
for i in range(10000000):
    client.stepSimulation()
    snake.update_virtual_chassis_frame()
    

    contacts = p.getContactPoints()
    if contacts:
        for contact in contacts:
            _, _, aId, bId, _, contactPosOnA, contactPosOnB, contactNormalOnB, *_ = contact


    thetas = snake.rolling_gait(t_sim)
    snake.set_motors(thetas)

    # snake.set_motors(desired_joint_ang)
    time.sleep(dt)
    t_sim += dt
    
client.disconnect()
