import math

import numpy as np
import pybullet as p
from utils import draw_frame, to_SE3, forward_kinematics

class SnakeRobot:
    def __init__(self, length, client, head_position, head_orientation, mode="position") -> None:
        """Initialise SnakeBot

        Args:
            length (int): Number of joints
            client (pybulletclient): Sim interface
            base_position (list): Start position of the snake
            base_orientation (list): Start orientation of the snake
            mode (str, optional): Joint interface mode (["position", "torque", "velocity"]). Defaults to "position".
        """
        self._length = length
        self._client = client
        self._mode = mode

        self.link_mass = 1
        self.link_length = 0.5
        self.lateral_friction = 5
        self.anistropic_friction = [1, 1, 1]
        self._snakeID = self.create_snake(head_position, head_orientation)

        self.debug_items = {}

        # Virtual chassis
        self.prev_V = None
        self.T_virtual_chassis_wrt_base = np.eye(4)
        self.T_body_to_world = np.eye(4)

    def create_snake(self, head_position, head_orientation):
        """Creates a snake multiBody object

        Returns:
            int: ID of the snake object
        """
        mass = 0.06
        sphereRadius = self.link_length / 2

        colBoxId = self._client.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[sphereRadius, sphereRadius, sphereRadius]
        )

        visualShapeIdRed = self._client.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[sphereRadius, sphereRadius, sphereRadius],
            rgbaColor=[1, 0, 0, 1],
        )

        visualShapeIdWhite = self._client.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[sphereRadius, sphereRadius, sphereRadius],
            rgbaColor=[0.949, 0.858, 0.670, 1],
        )

        link_Masses = []
        linkCollisionShapeIndices = []
        linkVisualShapeIndices = []
        linkPositions = []
        linkOrientations = []
        linkInertialFramePositions = []
        linkInertialFrameOrientations = []
        indices = []
        jointTypes = []
        axis = []

        for i in range(self._length):
            link_Masses.append(self.link_mass)
            linkCollisionShapeIndices.append(colBoxId)
            linkVisualShapeIndices.append(visualShapeIdWhite)
            linkPositions.append([0, self.link_length, 0])
            linkOrientations.append([0, 0, 0, 1])
            linkInertialFramePositions.append([0, 0, 0])
            linkInertialFrameOrientations.append([0, 0, 0, 1])
            indices.append(i)
            jointTypes.append(p.JOINT_REVOLUTE)

            if i % 2 == 0:
                rotation_axis = [0, 0, 1]
            else:
                rotation_axis = [1, 0, 0]

            axis.append(rotation_axis)

        uid = self._client.createMultiBody(
            mass,
            colBoxId,
            visualShapeIdRed,
            head_position,
            head_orientation,
            linkMasses=link_Masses,
            linkCollisionShapeIndices=linkCollisionShapeIndices,
            linkVisualShapeIndices=linkVisualShapeIndices,
            linkPositions=linkPositions,
            linkOrientations=linkOrientations,
            linkInertialFramePositions=linkInertialFramePositions,
            linkInertialFrameOrientations=linkInertialFrameOrientations,
            linkParentIndices=indices,
            linkJointTypes=jointTypes,
            linkJointAxis=axis,
        )

        anistropicFriction = self.anistropic_friction
        lateralFriction = self.lateral_friction
        self._client.changeDynamics(uid, -1, lateralFriction=lateralFriction, anisotropicFriction=anistropicFriction)

        for i in range(self._client.getNumJoints(uid)):
            self._client.changeDynamics(uid, i, lateralFriction=lateralFriction, anisotropicFriction=anistropicFriction)

        return uid
    
    def set_motors(self, action):
        """Set joint motors

        Args:
            action (list): Set value for each joint
        """

        for joint in range(self._client.getNumJoints(self._snakeID)):
            if self._mode == "torque":
                self._client.setJointMotorControl2(
                    self._snakeID,
                    joint,
                    p.TORQUE_CONTROL,
                    force=action[joint],
                )  # Apply torque to the motor
            elif self._mode == "position":
                self._client.setJointMotorControl2(
                    self._snakeID,
                    joint,
                    p.POSITION_CONTROL,
                    # Set servo to a position
                    targetPosition=action[joint],
                    force=100,
                )
            elif self._mode == "velocity":
                self._client.setJointMotorControl2(
                    self._snakeID,
                    joint,
                    p.POSITION_CONTROL,
                    # Give certain velocity to servo
                    targetVelocity=action[joint],
                    maxVelocity=10,
                )

    def check_fwd_kinematics(self):
        joint_angles = self.get_joint_angles()
        num_total_links = self._length + 1
        for i in range(num_total_links):
            T_link_wrt_body = forward_kinematics(i, self.link_length, joint_angles).transform()

            T_FK_link_wrt_wrld = self.T_body_to_world @ T_link_wrt_body
            
            frame_name = f"FK_link_{i}"
            draw_frame(self._client, self.debug_items, frame_name, T_FK_link_wrt_wrld)
            # draw_frame(self._client, self.debug_items, "HEAD", self.T_body_to_world)
            # Get ground truth transformation of this link from pybullet

    def get_joint_angles(self):
        joint_data = np.array(
            self._client.getJointStates(self._snakeID, list(range(self._client.getNumJoints(self._snakeID)))),
            dtype=object
        )[:, 0]
        return joint_data

    def get_joint_data(self):
        """Returns the current state of the snake

        Returns:
            np.array: Current state observation
        """
        position, _ = self._client.getBasePositionAndOrientation(self._snakeID)
        obs = np.array(position[:2])  # (snake x, snake y)

        joint_data = np.array(
            self._client.getJointStates(self._snakeID, list(range(self._client.getNumJoints(self._snakeID)))),
            dtype=object
        )

        # (pos, speed) for each joint
        joint_positions = joint_data[:, 0:2]

        # (snake x, snake y, torque1, ... , torquen, pos1, ..., posn, speed1, ..., speedn)
        obs = np.append(obs, joint_positions)

        return obs.astype(np.float32)

    def get_imu_data(self, add_noise=False):
        """Reads in the IMU data for each link. The accelerometer does not have gravity in it. 
            No need to subtract out gravity as, when snake is still, accelerometers read zero.

        Returns:
            list: Nx6 numpy array where each row is [[lin_acc], [ang_vel]] 
        """
        num_links = self._client.getNumJoints(self._snakeID)

        imu_data = np.empty((0, 6))
        # Iterate over each link
        for link_idx in range(num_links):
            lin_acc, ang_vel = p.getLinkState(self._snakeID, link_idx, computeLinkVelocity=True)[6:8]

            if add_noise:
                lin_acc_std_dev = 0
                ang_vel_std_dev = 0
                lin_acc_noise = np.random.normal(0, lin_acc_std_dev, size=3)
                ang_vel_noise = np.random.normal(0, ang_vel_std_dev, size=3)
                lin_acc = lin_acc + lin_acc_noise
                ang_vel = ang_vel + ang_vel_noise

            curr_imu_data = np.hstack((lin_acc, ang_vel))
            imu_data = np.vstack((imu_data, curr_imu_data))
        
        return imu_data

    def update_virtual_chassis_frame(self, debug=True):
        """ Updates the position of the virtual chassis with respect to the body frame (self.T_virtual_chassis_wrt_base)
        and also updates the transformation of the body to the world (self.T_body_to_world)

        Args:
            debug (bool): Specifies if virtual chassis axes should be visualized
        """
        # Get the transformation matrix from the body frame --> the world frame
        body_position, body_orientation = p.getBasePositionAndOrientation(self._snakeID)
        self.T_body_to_world = to_SE3(body_position, body_orientation)

        # STEP 1: calculate geometric center of mass (IN THE INITIAL FRAME)
        link_positions = np.zeros((1, 3))

        num_links = self._client.getNumJoints(self._snakeID)
        for link_idx in range(num_links):
            # Get the link state (position and orientation) relative to the base link
            link_state = p.getLinkState(self._snakeID, link_idx, computeLinkVelocity=False, computeForwardKinematics=True)
            link_position_rel_world = np.array([[link_state[0][0], link_state[0][1], link_state[0][2],  1]]).T

            link_position_rel_base = np.linalg.inv(self.T_body_to_world) @ link_position_rel_world

            link_positions = np.vstack((link_positions, link_position_rel_base.T[:, 0:3]))

        center_of_mass_wrt_base = np.mean(link_positions, axis=0).reshape(1, 3)

        P = link_positions - center_of_mass_wrt_base


        # STEP 2: find the principle axes of rotation
        U, S, VT = np.linalg.svd(P, full_matrices=False)
        V = VT.T

        if self.prev_V is not None:
            dot_product = np.dot(self.prev_V[:, 0], V[:, 0])
            if dot_product < 0:
                V[:, 0] *= -1
                
            # Ensure positive dot product between the second singular vectors
            dot_product = np.dot(self.prev_V[:, 1], V[:, 1])
            if dot_product < 0:
                V[:, 1] *= -1

        # Modify V so it is right handed
        cross_product = np.cross(V[:, 0], V[:, 1])
        V[:, 2] = cross_product
        
        # Update virtual chassis transformation
        self.T_virtual_chassis_wrt_base = np.eye(4)
        self.T_virtual_chassis_wrt_base[:3, :3] = V
        self.T_virtual_chassis_wrt_base[:3, 3] = center_of_mass_wrt_base

        self.prev_V = V

        if debug:
            T_virtual_chassis_wrt_world = self.T_body_to_world @ self.T_virtual_chassis_wrt_base
            draw_frame(self._client, self.debug_items, "Virtual Chassis", T_virtual_chassis_wrt_world)
    