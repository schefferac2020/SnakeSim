import numpy as np
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc

from SnakeRobot import SnakeRobot

# Setup pybullet client
client = bc.BulletClient(connection_mode=p.GUI)
client.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
client.setGravity(0, 0, -10)

# Make the snake
snake = SnakeRobot(10, client, [0, 0, 0], [0, 0, 0, 1])

# Make the rest of the world
planeId = client.loadURDF("plane.urdf")
startPos = [0, 0, 1]
startOrientation = client.getQuaternionFromEuler([0, 0, 0])
# set the center of mass frame (loadURDF sets base link frame) startPos/OrnclientresetBasePositionAndOrientation(boxId, startPos, startOrientation)

client.setRealTimeSimulation(1)

desired_joint_ang = 0.5 * np.ones(10)

# Simulate
for i in range(10000000):
    client.stepSimulation()

    move = snake.generate_sin_move()
    snake.set_motors(move)
    # snake.set_motors(desired_joint_ang)
    # print("State is ", snake.get_state())
    # time.sleep(1./240.)
client.disconnect()
