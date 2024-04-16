import time
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
from manifpy import SE3Tangent

from ekf import EKF
from particle_filter import TerrainParticleFilter
from snake_controller import SnakeController
from snake_robot import SnakeRobot
from terrain import Terrain
from utils import *


def run():
    dt = 1. / 120.

    # Setup pybullet client
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(0)
    p.setTimeStep(dt)

    # Make the terrain
    # terrain = Terrain()
    p.loadURDF("plane.urdf")

    # Make the snake
    N = 8  # links other than the red head
    link_length = 0.5
    snake = SnakeRobot(N, link_length, [0, 0, 5], [0, 0, 0, 1])
    controller = SnakeController(N)

    ekf = EKF(N, link_length)
    ekf.state.w = np.array([0.0, 0., 0.0])

    # Initialize q to be the start q of snake virtual chasis
    snake.update_virtual_chassis_frame()
    T_virtual_chassis_wrt_world = snake.T_body_to_world @ snake.T_VC_to_head
    q_virutal_wrt_world = R_to_q(T_virtual_chassis_wrt_world[:3,:3])
    ekf.state.q = np.array(q_virutal_wrt_world)


    # pf = TerrainParticleFilter(N, 100, terrain)

    forward_cmd = 0
    turn_cmd = 0

    # data for plotting
    accel_data = []
    gyro_data = []
    vel_data = []
    cmd_angle_data = []
    enc_data = []

    # Simulate
    t_sim = 0
    while p.isConnected():
        p.stepSimulation()
        snake.update_virtual_chassis_frame()
        # snake.check_fwd_kinematics()

        keys = p.getKeyboardEvents()
        for k, v in keys.items():
            if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED):
                forward_cmd += 0.5
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED):
                forward_cmd -= 0.5
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                turn_cmd += 0.5
            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                turn_cmd -= 0.5
            if k == p.B3G_SPACE and (v & p.KEY_WAS_TRIGGERED):
                forward_cmd = 0
                turn_cmd = 0

        # angles = controller.rolling_gait(t_sim)
        angles = controller.inchworm_gait(t_sim, 5 * forward_cmd, -0.2 * turn_cmd)
        # angles = [0, 0.5]* 8
        snake.set_motors(angles)
        cmd_angle_data.append(angles)

        # Prediction step of the EKF
        ekf.set_VC_Transform(snake.T_VC_to_head)
        ekf.predict(dt)

        VC_pos = (snake.T_body_to_world @ snake.T_VC_to_head)[0:3, 3]
        ekf_transform = to_SE3(np.array(VC_pos), ekf.state.q)
        draw_frame(snake.debug_items, "EKF_PREDICTION_STEP", ekf_transform)

        # Get Measurements
        encoders = snake.get_joint_angles()
        accelerometers, gyros, velocities = snake.get_imu_data(dt, debug=True)
        accel_data.append(accelerometers)
        gyro_data.append(gyros)
        vel_data.append(velocities)
        enc_data.append(encoders)

        # Update Step of EKF
        # ekf.update(encoders, accelerometers, gyros, dt)

        # Prediction step of the PF
        orientation = make_so3_nonstupid(ekf.state.q)
        twist = SE3Tangent(np.array([forward_cmd, 0, 0, 0, 0, turn_cmd]))
        # pf.prediction(orientation, twist)

        # TODO: make this a snake function
        contact_normals = []
        contacts = p.getContactPoints()
        if contacts:
            for contact in contacts:
                _, _, a_id, b_id, _, contact_position_on_a, contact_position_on_b, contact_normal_on_b, *_ = contact
                # Collider A always seems to have the lower id
                # I think the terrain is 0 as it is added first
                if a_id == 0:
                    contact_normals.append((b_id, contact_normal_on_b))

        # pf.correction(contact_normals)

        # doink = pf.filter()
        # print(doink)

        time.sleep(dt)
        t_sim += dt
        # print(t_sim)

    accel_data = np.array(accel_data)
    ts = np.linspace(0, t_sim, accel_data.shape[0])
    plot_accel(ts, accel_data, np.arange(N + 1))
    
    gyro_data = np.array(gyro_data)
    plot_gyro(ts, gyro_data, np.arange(N + 1))
    
    vel_data = np.array(vel_data)
    plot_vel(ts, vel_data, np.arange(N + 1))
    
    enc_data = np.array(enc_data)
    cmd_angle_data = np.array(cmd_angle_data)
    plot_joint_angles(ts, cmd_angle_data, enc_data, np.arange(N))
    
    p.disconnect()


if __name__ == "__main__":
    run()
