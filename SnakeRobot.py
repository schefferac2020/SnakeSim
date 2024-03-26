import math

import numpy as np
import pybullet as p


class SnakeRobot:
    def __init__(self, length, client, base_position, base_orientation, mode="position") -> None:
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
        self._snakeID = self.create_snake(base_position, base_orientation)
        print("Snake ID is", self._snakeID)
        self._mode = mode

        # for manual sin move
        self.scaleStart = 1.0
        self.m_waveFront = 0.0

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

    def get_imu_data(self):
        pass

    def get_ids(self):
        """get snake id and client id

        Returns:
            (int, int): Id of snake and id of client
        """
        return self._snakeID, self._client._client

    def create_snake(self, base_position, base_orientation):
        """Creates a snake multiBody object

        Returns:
            int: ID of the snake object
        """
        mass = 0.06
        sphereRadius = 0.25

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
            link_Masses.append(1)
            linkCollisionShapeIndices.append(colBoxId)
            linkVisualShapeIndices.append(visualShapeIdWhite)
            linkPositions.append([0, sphereRadius * 2.0 + 0.01, 0])
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

        # Draw Debug rotation axes
        # for i in range(1, self._length):
        #     joint_position = (np.array(linkPositions[i-1]) + np.array(linkPositions[i])) / 2
        #     joint_axis = axis[i]

        #     axis_length = 0.5
        #     end_point = [
        #         joint_position[0] + axis_length * joint_axis[0],
        #         joint_position[1] + axis_length * joint_axis[1],
        #         joint_position[2] + axis_length * joint_axis[2],
        #     ]

        #     # Draw the debug line
        #     self._client.addUserDebugLine(
        #         lineFromXYZ=joint_position,
        #         lineToXYZ=end_point,
        #         lineColorRGB=joint_axis,
        #         lineWidth=3,
        #     )

        uid = self._client.createMultiBody(
            mass,
            colBoxId,
            visualShapeIdRed,
            base_position,
            base_orientation,
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

        anistropicFriction = [1, 0.01, 0.01]
        lateralFriction = 5
        self._client.changeDynamics(uid, -1, lateralFriction=lateralFriction, anisotropicFriction=anistropicFriction)

        for i in range(self._client.getNumJoints(uid)):
            self._client.changeDynamics(uid, i, lateralFriction=lateralFriction, anisotropicFriction=anistropicFriction)

        return uid

    def generate_sin_move(self):
        """Manual movement function mimicking a sin wave function

        Returns:
            list: Joint positions
        """
        dt = 1. / 240.  # simulator step size
        m_waveLength = 4
        m_wavePeriod = 1.5
        m_waveAmplitude = 0.4

        m_segmentLength = 0.25 * 2.0

        # start of the snake with smaller waves.
        if (self.m_waveFront < m_segmentLength * 4.0):
            self.scaleStart = self.m_waveFront / (m_segmentLength * 4.0)

        moves = []
        for joint in range(self._client.getNumJoints(self._snakeID)):
            segment = joint  # numMuscles-1-joint

            # map segment to phase
            phase = (self.m_waveFront - (segment + 1)
                     * m_segmentLength) / m_waveLength
            phase -= math.floor(phase)
            phase *= math.pi * 2.0

            # map phase to curvature
            targetPos = math.sin(phase) * self.scaleStart * m_waveAmplitude

            moves.append(targetPos)

        # wave keeps track of where the wave is in time
        self.m_waveFront += dt / m_wavePeriod * m_waveLength
        return moves
