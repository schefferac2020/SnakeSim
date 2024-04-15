import numpy as np
import pybullet as p
import pybullet_data
from manifpy import SE3Tangent

from ekf import EKF
from snake_controller import SnakeController
from snake_robot import SnakeRobot
from utils import draw_frame, to_SE3, make_so3_nonstupid


def run():
    dt = 1. / 60.

    # Setup pybullet client
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(1)
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
    ekf.state.w = np.array([0, 0.5, 0])

    # pf = TerrainParticleFilter(100, N, terrain)

    forward_cmd = 0
    turn_cmd = 0

    # Simulate
    t_sim = 0
    while True:

        p.stepSimulation()
        snake.update_virtual_chassis_frame()

        snake.check_fwd_kinematics()

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
        # angles = controller.inchworm_s_gait(t_sim, 10*forward_cmd, 0.5)

        # angles = [0, 0.5]* 8
        snake.set_motors(angles)

        # Prediction step of the EKF
        ekf.set_VC_Transform(snake.T_virtual_chassis_wrt_base)

        ekf.predict(dt)

        VC_pos = (snake.T_body_to_world @ snake.T_virtual_chassis_wrt_base)[0:3, 3]
        ekf_transform = to_SE3(np.array(VC_pos), ekf.state.q)
        draw_frame(snake.debug_items, "EKF_PREDICTION_STEP", ekf_transform)

        # Get Measuremnts
        encoders = snake.get_joint_angles()
        # print(encoders)
        [accelerometers, gyros] = snake.get_imu_data(dt, debug=True)

        # print(accelerometers)
        # print(gyros)
        # print(snake.get_imu_data())

        # Update Step of EKF
        # ekf.update(encoders, accelerometers, gyros, dt)

        # Prediction step of the PF
        orientation = make_so3_nonstupid(ekf.state.q)
        twist = SE3Tangent(np.array([forward_cmd, 0, 0, 0, 0, turn_cmd]))
        # pf.prediction(orientation, twist)

        # TODO: make this a snake function
        contacts = p.getContactPoints()
        if contacts:
            for contact in contacts:
                _, _, a_id, b_id, _, contact_position_on_a, contact_position_on_b, contact_normal_on_b, *_ = contact

        # time.sleep(dt)
        t_sim += dt
        print(t_sim)

    p.disconnect()


if __name__ == "__main__":
    run()
