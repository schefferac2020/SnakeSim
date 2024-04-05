
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc

from SnakeRobot import SnakeRobot
from Terrain import Terrain
from snake_controller import SnakeController

import time


def run():
    dt = 1. / 240.

    # Setup pybullet client
    client = bc.BulletClient(connection_mode=p.GUI)
    client.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    client.setGravity(0, 0, -10)
    client.setRealTimeSimulation(1)
    client.setTimeStep(dt)

    # Make the terrain
    terrain = Terrain(client)
    # planeId = client.loadURDF("plane.urdf")

    # Make the snake
    N = 16 # links other than the red head
    snake = SnakeRobot(N, client, [0, 0, 5], [0, 0, 0, 1])
    controller = SnakeController(N)

    forward = 0
    turn = 0

    # Simulate
    t_sim = 0
    for i in range(10000000):
        client.stepSimulation()
        snake.update_virtual_chassis_frame()
        
        # TODO: make this a snake function
        contacts = p.getContactPoints()
        if contacts:
            for contact in contacts:
                _, _, aId, bId, _, contactPosOnA, contactPosOnB, contactNormalOnB, *_ = contact

        keys = client.getKeyboardEvents()
        for k, v in keys.items():
            if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED):
                forward = 1
            # if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED):
            #     forward = 0
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED):
                forward = -1
            # if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED):
            #     forward = 0
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                turn = 1
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED):
                turn = 0
            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                turn = -1
            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED):
                turn = 0
            if k == p.B3G_SPACE and (v & p.KEY_WAS_TRIGGERED):
                forward = 0
        print(forward, turn)
        # angles = controller.rolling_gait(t_sim)
        angles = controller.inchworm_gait(t_sim, 10*forward, -0.2*turn)
        snake.set_motors(angles)

        time.sleep(dt)
        t_sim += dt
        
    client.disconnect()


if __name__ == "__main__":
    run()
