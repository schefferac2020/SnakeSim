# encoding: utf-8
# module pybullet
# from /home/quintin/Programming/snake-sim/venv/lib/python3.10/site-packages/pybullet.cpython-310-x86_64-linux-gnu.so
# by generator 1.147
""" Python bindings for Bullet Physics Robotics API (also known as Shared Memory API) """
# no imports

# Variables with simple values

ACTIVATION_STATE_DISABLE_SLEEPING = 2
ACTIVATION_STATE_DISABLE_WAKEUP = 32

ACTIVATION_STATE_ENABLE_SLEEPING = 1
ACTIVATION_STATE_ENABLE_WAKEUP = 16

ACTIVATION_STATE_SLEEP = 8

ACTIVATION_STATE_WAKE_UP = 4

AddFileIOAction = 1024

B3G_ALT = 65308
B3G_BACKSPACE = 65305
B3G_CONTROL = 65307
B3G_DELETE = 65304

B3G_DOWN_ARROW = 65298

B3G_END = 65301
B3G_F1 = 65280
B3G_F10 = 65289
B3G_F11 = 65290
B3G_F12 = 65291
B3G_F13 = 65292
B3G_F14 = 65293
B3G_F15 = 65294
B3G_F2 = 65281
B3G_F3 = 65282
B3G_F4 = 65283
B3G_F5 = 65284
B3G_F6 = 65285
B3G_F7 = 65286
B3G_F8 = 65287
B3G_F9 = 65288
B3G_HOME = 65302
B3G_INSERT = 65303

B3G_LEFT_ARROW = 65295

B3G_PAGE_DOWN = 65300
B3G_PAGE_UP = 65299

B3G_RETURN = 65309

B3G_RIGHT_ARROW = 65296

B3G_SHIFT = 65306
B3G_SPACE = 32

B3G_UP_ARROW = 65297

CNSFileIO = 3

CONSTRAINT_SOLVER_LCP_DANTZIG = 3
CONSTRAINT_SOLVER_LCP_PGS = 2
CONSTRAINT_SOLVER_LCP_SI = 1

CONTACT_RECOMPUTE_CLOSEST = 1

CONTACT_REPORT_EXISTING = 0

COV_ENABLE_DEPTH_BUFFER_PREVIEW = 14

COV_ENABLE_GUI = 1

COV_ENABLE_KEYBOARD_SHORTCUTS = 9

COV_ENABLE_MOUSE_PICKING = 10

COV_ENABLE_PLANAR_REFLECTION = 16

COV_ENABLE_RENDERING = 7

COV_ENABLE_RGB_BUFFER_PREVIEW = 13

COV_ENABLE_SEGMENTATION_MARK_PREVIEW = 15

COV_ENABLE_SHADOWS = 2

COV_ENABLE_SINGLE_STEP_RENDERING = 17

COV_ENABLE_TINY_RENDERER = 12

COV_ENABLE_VR_PICKING = 5

COV_ENABLE_VR_RENDER_CONTROLLERS = 6

COV_ENABLE_VR_TELEPORTING = 4

COV_ENABLE_WIREFRAME = 3

COV_ENABLE_Y_AXIS_UP = 11

DIRECT = 2

ER_BULLET_HARDWARE_OPENGL = 131072

ER_NO_SEGMENTATION_MASK = 4

ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX = 1

ER_TINY_RENDERER = 65536

ER_USE_PROJECTIVE_TEXTURE = 2

GEOM_BOX = 3
GEOM_CAPSULE = 7

GEOM_CONCAVE_INTERNAL_EDGE = 2

GEOM_CYLINDER = 4

GEOM_FORCE_CONCAVE_TRIMESH = 1

GEOM_HEIGHTFIELD = 9
GEOM_MESH = 5
GEOM_PLANE = 6
GEOM_SPHERE = 2

GRAPHICS_CLIENT = 14
GRAPHICS_SERVER = 15

GRAPHICS_SERVER_MAIN_THREAD = 17

GRAPHICS_SERVER_TCP = 16

GUI = 1

GUI_MAIN_THREAD = 8

GUI_SERVER = 7

IK_DLS = 0

IK_HAS_JOINT_DAMPING = 128

IK_HAS_NULL_SPACE_VELOCITY = 64

IK_HAS_TARGET_ORIENTATION = 32
IK_HAS_TARGET_POSITION = 16

IK_SDLS = 1

JOINT_FEEDBACK_IN_JOINT_FRAME = 2

JOINT_FEEDBACK_IN_WORLD_SPACE = 1

JOINT_FIXED = 4
JOINT_GEAR = 6
JOINT_PLANAR = 3
JOINT_POINT2POINT = 5
JOINT_PRISMATIC = 1
JOINT_REVOLUTE = 0
JOINT_SPHERICAL = 2

KEY_IS_DOWN = 1

KEY_WAS_RELEASED = 4
KEY_WAS_TRIGGERED = 2

LINK_FRAME = 1

MAX_RAY_INTERSECTION_BATCH_SIZE = 16384

MESH_DATA_SIMULATION_MESH = 1

MJCF_COLORS_FROM_FILE = 512

PD_CONTROL = 3

POSITION_CONTROL = 2

PosixFileIO = 1

RemoveFileIOAction = 1025

RESET_USE_DEFORMABLE_WORLD = 1

RESET_USE_DISCRETE_DYNAMICS_WORLD = 2

RESET_USE_REDUCED_DEFORMABLE_WORLD = 8

RESET_USE_SIMPLE_BROADPHASE = 4

SENSOR_FORCE_TORQUE = 1

SHARED_MEMORY = 3

SHARED_MEMORY_GUI = 14
SHARED_MEMORY_KEY = 12347
SHARED_MEMORY_KEY2 = 12348
SHARED_MEMORY_SERVER = 9

STABLE_PD_CONTROL = 4

STATE_LOGGING_ALL_COMMANDS = 7

STATE_LOGGING_CONTACT_POINTS = 5

STATE_LOGGING_CUSTOM_TIMER = 9

STATE_LOGGING_GENERIC_ROBOT = 1

STATE_LOGGING_MINITAUR = 0

STATE_LOGGING_PROFILE_TIMINGS = 6

STATE_LOGGING_VIDEO_MP4 = 3

STATE_LOGGING_VR_CONTROLLERS = 2

STATE_LOG_JOINT_MOTOR_TORQUES = 1

STATE_LOG_JOINT_TORQUES = 3

STATE_LOG_JOINT_USER_TORQUES = 2

STATE_REPLAY_ALL_COMMANDS = 8

TCP = 5

TORQUE_CONTROL = 1

UDP = 4

URDF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024

URDF_ENABLE_SLEEPING = 2048
URDF_ENABLE_WAKEUP = 262144

URDF_GLOBAL_VELOCITIES_MB = 256

URDF_GOOGLEY_UNDEFINED_COLORS = 8388608

URDF_IGNORE_COLLISION_SHAPES = 2097152

URDF_IGNORE_VISUAL_SHAPES = 1048576

URDF_INITIALIZE_SAT_FEATURES = 4096

URDF_MAINTAIN_LINK_ORDER = 131072

URDF_MERGE_FIXED_LINKS = 524288

URDF_PRINT_URDF_INFO = 4194304

URDF_USE_IMPLICIT_CYLINDER = 128

URDF_USE_INERTIA_FROM_FILE = 2

URDF_USE_MATERIAL_COLORS_FROM_MTL = 32768

URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536

URDF_USE_SELF_COLLISION = 8

URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32

URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16

URDF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192

VELOCITY_CONTROL = 0

VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS = 1

VISUAL_SHAPE_DOUBLE_SIDED = 4

VR_BUTTON_IS_DOWN = 1

VR_BUTTON_WAS_RELEASED = 4
VR_BUTTON_WAS_TRIGGERED = 2

VR_CAMERA_TRACK_OBJECT_ORIENTATION = 1

VR_DEVICE_CONTROLLER = 1

VR_DEVICE_GENERIC_TRACKER = 4

VR_DEVICE_HMD = 2

VR_MAX_BUTTONS = 64
VR_MAX_CONTROLLERS = 8

WORLD_FRAME = 2

ZipFileIO = 2

# functions

def addUserData(bodyUniqueId, key, value, linkIndex=-1, visualShapeIndex=-1, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    addUserData(bodyUniqueId, key, value, linkIndex=-1, visualShapeIndex=-1, physicsClientId=0)
    Adds or updates a user data entry. Returns user data identifier.
    """
    pass

def addUserDebugLine(*args, **kwargs): # real signature unknown
    """ Add a user debug draw line with lineFrom[3], lineTo[3], lineColorRGB[3], lineWidth, lifeTime. A lifeTime of 0 means permanent until removed. Returns a unique id for the user debug item. """
    pass

def addUserDebugParameter(*args, **kwargs): # real signature unknown
    """ Add a user debug parameter, such as a slider, that can be controlled using a GUI. """
    pass

def addUserDebugPoints(*args, **kwargs): # real signature unknown
    """ Add user debug draw points with pointPositions[3], pointColorsRGB[3], pointSize, lifeTime. A lifeTime of 0 means permanent until removed. Returns a unique id for the user debug item. """
    pass

def addUserDebugText(*args, **kwargs): # real signature unknown
    """ Add a user debug draw line with text, textPosition[3], textSize and lifeTime in seconds A lifeTime of 0 means permanent until removed. Returns a unique id for the user debug item. """
    pass

def applyExternalForce(*args, **kwargs): # real signature unknown
    """ for objectUniqueId, linkIndex (-1 for base/root link), apply a force [x,y,z] at the a position [x,y,z], flag to select FORCE_IN_LINK_FRAME or WORLD_FRAME coordinates """
    pass

def applyExternalTorque(*args, **kwargs): # real signature unknown
    """ for objectUniqueId, linkIndex (-1 for base/root link) apply a torque [x,y,z] in Cartesian coordinates, flag to select TORQUE_IN_LINK_FRAME or WORLD_FRAME coordinates """
    pass

def calculateInverseDynamics(*args, **kwargs): # real signature unknown
    """ Given an object id, joint positions, joint velocities and joint accelerations, compute the joint forces using Inverse Dynamics """
    pass

def calculateInverseKinematics(*args, **kwargs): # real signature unknown
    """ Inverse Kinematics bindings: Given an object id, current joint positions and target position for the end effector,compute the inverse kinematics and return the new joint state """
    pass

def calculateInverseKinematics2(*args, **kwargs): # real signature unknown
    """ Inverse Kinematics bindings: Given an object id, current joint positions and target positions for the end effectors,compute the inverse kinematics and return the new joint state """
    pass

def calculateJacobian(bodyUniqueId, linkIndex, localPosition, objPositions, objVelocities, objAccelerations, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    linearJacobian, angularJacobian = calculateJacobian(bodyUniqueId, linkIndex, localPosition, objPositions, objVelocities, objAccelerations, physicsClientId=0)
    Compute the jacobian for a specified local position on a body and its kinematics.
    Args:
      bodyIndex - a scalar defining the unique object id.
      linkIndex - a scalar identifying the link containing the local point.
      localPosition - a list of [x, y, z] of the coordinates defined in the link frame.
      objPositions - a list of the joint positions.
      objVelocities - a list of the joint velocities.
      objAccelerations - a list of the joint accelerations.
    Returns:
      linearJacobian - a list of the partial linear velocities of the jacobian.
      angularJacobian - a list of the partial angular velocities of the jacobian.
    """
    pass

def calculateMassMatrix(bodyUniqueId, objPositions, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    massMatrix = calculateMassMatrix(bodyUniqueId, objPositions, physicsClientId=0)
    Compute the mass matrix for an object and its chain of bodies.
    Args:
      bodyIndex - a scalar defining the unique object id.
      objPositions - a list of the joint positions.
    Returns:
      massMatrix - a list of lists of the mass matrix components.
    """
    pass

def calculateVelocityQuaternion(*args, **kwargs): # real signature unknown
    """ Compute the angular velocity given start and end quaternion and delta time. """
    pass

def changeConstraint(*args, **kwargs): # real signature unknown
    """ Change some parameters of an existing constraint, such as the child pivot or child frame orientation, using its unique id. """
    pass

def changeDynamics(*args, **kwargs): # real signature unknown
    """ change dynamics information such as mass, lateral friction coefficient. """
    pass

def changeTexture(*args, **kwargs): # real signature unknown
    """ Change a texture file. """
    pass

def changeVisualShape(*args, **kwargs): # real signature unknown
    """ Change part of the visual shape information for one object. """
    pass

def computeDofCount(*args, **kwargs): # real signature unknown
    """ computeDofCount returns the number of degrees of freedom, including 7 degrees of freedom for the base in case of floating base """
    pass

def computeProjectionMatrix(*args, **kwargs): # real signature unknown
    """ Compute a camera projection matrix from screen left/right/bottom/top/near/far values """
    pass

def computeProjectionMatrixFOV(*args, **kwargs): # real signature unknown
    """ Compute a camera projection matrix from fov, aspect ratio, near, far values """
    pass

def computeViewMatrix(*args, **kwargs): # real signature unknown
    """ Compute a camera viewmatrix from camera eye,  target position and up vector """
    pass

def computeViewMatrixFromYawPitchRoll(*args, **kwargs): # real signature unknown
    """ Compute a camera viewmatrix from camera eye,  target position and up vector """
    pass

def configureDebugVisualizer(*args, **kwargs): # real signature unknown
    """ For the 3D OpenGL Visualizer, enable/disable GUI, shadows. """
    pass

def connect(method, key=None, options=''): # real signature unknown; restored from __doc__
    """
    connect(method, key=SHARED_MEMORY_KEY, options='')
    connect(method, hostname='localhost', port=1234, options='')
    Connect to an existing physics server (using shared memory by default).
    """
    pass

def createCollisionShape(*args, **kwargs): # real signature unknown
    """ Create a collision shape. Returns a non-negative (int) unique id, if successfull, negative otherwise. """
    pass

def createCollisionShapeArray(*args, **kwargs): # real signature unknown
    """ Create collision shapes. Returns a non-negative (int) unique id, if successfull, negative otherwise. """
    pass

def createConstraint(*args, **kwargs): # real signature unknown
    """ Create a constraint between two bodies. Returns a (int) unique id, if successfull. """
    pass

def createMultiBody(*args, **kwargs): # real signature unknown
    """ Create a multi body. Returns a non-negative (int) unique id, if successfull, negative otherwise. """
    pass

def createSoftBodyAnchor(*args, **kwargs): # real signature unknown
    """ Create an anchor (attachment) between a soft body and a rigid or multi body. """
    pass

def createVisualShape(*args, **kwargs): # real signature unknown
    """ Create a visual shape. Returns a non-negative (int) unique id, if successfull, negative otherwise. """
    pass

def createVisualShapeArray(*args, **kwargs): # real signature unknown
    """ Create visual shapes. Returns a non-negative (int) unique id, if successfull, negative otherwise. """
    pass

def disconnect(physicsClientId=0): # real signature unknown; restored from __doc__
    """
    disconnect(physicsClientId=0)
    Disconnect from the physics server.
    """
    pass

def enableJointForceTorqueSensor(*args, **kwargs): # real signature unknown
    """ Enable or disable a joint force/torque sensor measuring the joint reaction forces. """
    pass

def executePluginCommand(*args, **kwargs): # real signature unknown
    """ Execute a command, implemented in a plugin. """
    pass

def getAABB(*args, **kwargs): # real signature unknown
    """ Get the axis aligned bound box min and max coordinates in world space. """
    pass

def getAPIVersion(*args, **kwargs): # real signature unknown
    """ Get version of the API. Compatibility exists for connections using the same API version. Make sure both client and server use the same number of bits (32-bit or 64bit). """
    pass

def getAxisAngleFromQuaternion(*args, **kwargs): # real signature unknown
    """ Compute the quaternion from axis and angle representation. """
    pass

def getAxisDifferenceQuaternion(*args, **kwargs): # real signature unknown
    """ Compute the velocity axis difference from two quaternions. """
    pass

def getBasePositionAndOrientation(*args, **kwargs): # real signature unknown
    """ Get the world position and orientation of the base of the object. (x,y,z) position vector and (x,y,z,w) quaternion orientation. """
    pass

def getBaseVelocity(*args, **kwargs): # real signature unknown
    """ Get the linear and angular velocity of the base of the object  in world space coordinates. (x,y,z) linear velocity vector and (x,y,z) angular velocity vector. """
    pass

def getBodyInfo(*args, **kwargs): # real signature unknown
    """ Get the body info, given a body unique id. """
    pass

def getBodyUniqueId(*args, **kwargs): # real signature unknown
    """ getBodyUniqueId is used after connecting to server with existing bodies.Get the unique id of the body, given a integer range [0.. number of bodies). """
    pass

def getCameraImage(*args, **kwargs): # real signature unknown
    """ Render an image (given the pixel resolution width, height, camera viewMatrix , projectionMatrix, lightDirection, lightColor, lightDistance, shadow, lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff, and renderer), and return the 8-8-8bit RGB pixel data and floating point depth values as NumPy arrays """
    pass

def getClosestPoints(*args, **kwargs): # real signature unknown
    """ Compute the closest points between two objects, if the distance is below a given threshold.Input is two objects unique ids and distance threshold. """
    pass

def getCollisionShapeData(*args, **kwargs): # real signature unknown
    """ Return the collision shape information for one object. """
    pass

def getConnectionInfo(physicsClientId=0): # real signature unknown; restored from __doc__
    """
    getConnectionInfo(physicsClientId=0)
    Return if a given client id is connected, and using what method.
    """
    pass

def getConstraintInfo(*args, **kwargs): # real signature unknown
    """ Get the user-created constraint info, given a constraint unique id. """
    pass

def getConstraintState(*args, **kwargs): # real signature unknown
    """ Get the user-created constraint state (applied forces), given a constraint unique id. """
    pass

def getConstraintUniqueId(*args, **kwargs): # real signature unknown
    """ Get the unique id of the constraint, given a integer index in range [0.. number of constraints). """
    pass

def getContactPoints(*args, **kwargs): # real signature unknown
    """ Return existing contact points after the stepSimulation command. Optional arguments one or two object unique ids, that need to be involved in the contact. """
    pass

def getDebugVisualizerCamera(*args, **kwargs): # real signature unknown
    """ Get information about the 3D visualizer camera, such as width, height, view matrix, projection matrix etc. """
    pass

def getDifferenceQuaternion(*args, **kwargs): # real signature unknown
    """ Compute the quaternion difference from two quaternions. """
    pass

def getDynamicsInfo(*args, **kwargs): # real signature unknown
    """ Get dynamics information such as mass, lateral friction coefficient. """
    pass

def getEulerFromQuaternion(*args, **kwargs): # real signature unknown
    """ Convert quaternion [x,y,z,w] to Euler [roll, pitch, yaw] as in URDF/SDF convention """
    pass

def getJointInfo(*args, **kwargs): # real signature unknown
    """ Get the name and type info for a joint on a body. """
    pass

def getJointState(*args, **kwargs): # real signature unknown
    """ Get the state (position, velocity etc) for a joint on a body. """
    pass

def getJointStateMultiDof(*args, **kwargs): # real signature unknown
    """ Get the state (position, velocity etc) for a joint on a body. (supports planar and spherical joints) """
    pass

def getJointStates(*args, **kwargs): # real signature unknown
    """ Get the state (position, velocity etc) for multiple joints on a body. """
    pass

def getJointStatesMultiDof(*args, **kwargs): # real signature unknown
    """ Get the states (position, velocity etc) for multiple joint on a body. (supports planar and spherical joints) """
    pass

def getKeyboardEvents(*args, **kwargs): # real signature unknown
    """ Get keyboard events, keycode and state (KEY_IS_DOWN, KEY_WAS_TRIGGERED, KEY_WAS_RELEASED) """
    pass

def getLinkState(objectUniqueId, linkIndex, computeLinkVelocity=0, computeForwardKinematics=0, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    position_linkcom_world, world_rotation_linkcom,
    position_linkcom_frame, frame_rotation_linkcom,
    position_frame_world, world_rotation_frame,
    linearVelocity_linkcom_world, angularVelocity_linkcom_world
      = getLinkState(objectUniqueId, linkIndex, computeLinkVelocity=0,
                     computeForwardKinematics=0, physicsClientId=0)
    Provides extra information such as the Cartesian world coordinates center of mass (COM) of the link, relative to the world reference frame.
    """
    pass

def getLinkStates(*args, **kwargs): # real signature unknown
    """ same as getLinkState except it takes a list of linkIndices """
    pass

def getMatrixFromQuaternion(*args, **kwargs): # real signature unknown
    """ Compute the 3x3 matrix from a quaternion, as a list of 9 values (row-major) """
    pass

def getMeshData(*args, **kwargs): # real signature unknown
    """ Get mesh data. Returns vertices etc from the mesh. """
    pass

def getMouseEvents(*args, **kwargs): # real signature unknown
    """ Get mouse events, event type and button state (KEY_IS_DOWN, KEY_WAS_TRIGGERED, KEY_WAS_RELEASED) """
    pass

def getNumBodies(*args, **kwargs): # real signature unknown
    """ Get the number of bodies in the simulation. """
    pass

def getNumConstraints(*args, **kwargs): # real signature unknown
    """ Get the number of user-created constraints in the simulation. """
    pass

def getNumJoints(*args, **kwargs): # real signature unknown
    """ Get the number of joints for an object. """
    pass

def getNumUserData(bodyUniqueId_physicsClientId=0): # real signature unknown; restored from __doc__
    """
    getNumUserData(bodyUniqueId physicsClientId=0)
    Retrieves the number of user data entries in a body.
    """
    pass

def getOverlappingObjects(*args, **kwargs): # real signature unknown
    """ Return all the objects that have overlap with a given axis-aligned bounding box volume (AABB).Input are two vectors defining the AABB in world space [min_x,min_y,min_z],[max_x,max_y,max_z]. """
    pass

def getPhysicsEngineParameters(*args, **kwargs): # real signature unknown
    """ Get the current values of internal physics engine parameters """
    pass

def getQuaternionFromAxisAngle(*args, **kwargs): # real signature unknown
    """ Compute the quaternion from axis and angle representation. """
    pass

def getQuaternionFromEuler(*args, **kwargs): # real signature unknown
    """ Convert Euler [roll, pitch, yaw] as in URDF/SDF convention, to quaternion [x,y,z,w] """
    pass

def getQuaternionSlerp(*args, **kwargs): # real signature unknown
    """ Compute the spherical interpolation given a start and end quaternion and an interpolation value in range [0..1] """
    pass

def getTetraMeshData(*args, **kwargs): # real signature unknown
    """ Get mesh data. Returns tetra from the mesh. """
    pass

def getUserData(userDataId, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    getUserData(userDataId, physicsClientId=0)
    Returns the user data value.
    """
    pass

def getUserDataId(bodyUniqueId, key, linkIndex=-1, visualShapeIndex=-1, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    getUserDataId(bodyUniqueId, key, linkIndex=-1, visualShapeIndex=-1, physicsClientId=0)
    Retrieves the userDataId given the key and optionally link and visual shape index.
    """
    pass

def getUserDataInfo(bodyUniqueId, userDataIndex, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    getUserDataInfo(bodyUniqueId, userDataIndex, physicsClientId=0)
    Retrieves the key and the identifier of a user data as (userDataId, key, bodyUniqueId, linkIndex, visualShapeIndex).
    """
    pass

def getVisualShapeData(*args, **kwargs): # real signature unknown
    """ Return the visual shape information for one object. """
    pass

def getVREvents(*args, **kwargs): # real signature unknown
    """ Get Virtual Reality events, for example to track VR controllers position/buttons """
    pass

def invertTransform(*args, **kwargs): # real signature unknown
    """ Invert a transform, provided as [position], [quaternion]. """
    pass

def isConnected(physicsClientId=0): # real signature unknown; restored from __doc__
    """
    isConnected(physicsClientId=0)
    Return if a given client id is connected.
    """
    pass

def isNumpyEnabled(*args, **kwargs): # real signature unknown
    """ return True if PyBullet was compiled with NUMPY support. This makes the getCameraImage API faster """
    pass

def loadBullet(*args, **kwargs): # real signature unknown
    """ Load a world from a .bullet file. """
    pass

def loadMJCF(*args, **kwargs): # real signature unknown
    """ Load multibodies from an MJCF file. """
    pass

def loadPlugin(*args, **kwargs): # real signature unknown
    """ Load a plugin, could implement custom commands etc. """
    pass

def loadSDF(*args, **kwargs): # real signature unknown
    """ Load multibodies from an SDF file. """
    pass

def loadSoftBody(*args, **kwargs): # real signature unknown
    """ Load a softbody from an obj file. """
    pass

def loadTexture(*args, **kwargs): # real signature unknown
    """ Load texture file. """
    pass

def loadURDF(fileName, basePosition=None, baseOrientation=None, useMaximalCoordinates=0, useFixedBase=0, flags=0, globalScaling=1.0, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    bodyUniqueId = loadURDF(fileName, basePosition=[0.,0.,0.], baseOrientation=[0.,0.,0.,1.], useMaximalCoordinates=0, useFixedBase=0, flags=0, globalScaling=1.0, physicsClientId=0)
    Create a multibody by loading a URDF file.
    """
    pass

def multiplyTransforms(*args, **kwargs): # real signature unknown
    """ Multiply two transform, provided as [position], [quaternion]. """
    pass

def performCollisionDetection(physicsClientId=0): # real signature unknown; restored from __doc__
    """
    performCollisionDetection(physicsClientId=0)
    Update AABBs, compute overlapping pairs and contact points. stepSimulation also includes this already.
    """
    pass

def rayTest(*args, **kwargs): # real signature unknown
    """ Cast a ray and return the first object hit, if any. Takes two arguments (from_position [x,y,z] and to_position [x,y,z] in Cartesian world coordinates """
    pass

def rayTestBatch(*args, **kwargs): # real signature unknown
    """ Cast a batch of rays and return the result for each of the rays (first object hit, if any. or -1) Takes two required arguments (list of from_positions [x,y,z] and a list of to_positions [x,y,z] in Cartesian world coordinates) and one optional argument numThreads to specify the number of threads to use to compute the ray intersections for the batch. Specify 0 to let Bullet decide, 1 (default) for single core execution, 2 or more to select the number of threads to use. """
    pass

def readUserDebugParameter(*args, **kwargs): # real signature unknown
    """ Read the current value of a user debug parameter, given the user debug item unique id. """
    pass

def removeAllUserDebugItems(*args, **kwargs): # real signature unknown
    """ remove all user debug draw items """
    pass

def removeAllUserParameters(*args, **kwargs): # real signature unknown
    """ remove all user debug parameters (sliders, buttons) """
    pass

def removeBody(*args, **kwargs): # real signature unknown
    """ Remove a body by its body unique id. """
    pass

def removeCollisionShape(*args, **kwargs): # real signature unknown
    """ Remove a collision shape. Only useful when the collision shape is not used in a body (to perform a getClosestPoint query). """
    pass

def removeConstraint(*args, **kwargs): # real signature unknown
    """ Remove a constraint using its unique id. """
    pass

def removeState(*args, **kwargs): # real signature unknown
    """ Remove a state created using saveState by its state unique id. """
    pass

def removeUserData(userDataId, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    removeUserData(userDataId, physicsClientId=0)
    Removes a user data entry.
    """
    pass

def removeUserDebugItem(*args, **kwargs): # real signature unknown
    """ remove a user debug draw item, giving its unique id """
    pass

def renderImage(*args, **kwargs): # real signature unknown
    """ obsolete, please use getCameraImage and getViewProjectionMatrices instead """
    pass

def resetBasePositionAndOrientation(*args, **kwargs): # real signature unknown
    """ Reset the world position and orientation of the base of the object instantaneously, not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation. """
    pass

def resetBaseVelocity(*args, **kwargs): # real signature unknown
    """ Reset the linear and/or angular velocity of the base of the object  in world space coordinates. linearVelocity (x,y,z) and angularVelocity (x,y,z). """
    pass

def resetDebugVisualizerCamera(*args, **kwargs): # real signature unknown
    """ For the 3D OpenGL Visualizer, set the camera distance, yaw, pitch and target position. """
    pass

def resetJointState(objectUniqueId, jointIndex, targetValue, targetVelocity=0, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    resetJointState(objectUniqueId, jointIndex, targetValue, targetVelocity=0, physicsClientId=0)
    Reset the state (position, velocity etc) for a joint on a body instantaneously, not through physics simulation.
    """
    pass

def resetJointStateMultiDof(objectUniqueId, jointIndex, targetValue, targetVelocity=0, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    resetJointStateMultiDof(objectUniqueId, jointIndex, targetValue, targetVelocity=0, physicsClientId=0)
    Reset the state (position, velocity etc) for a joint on a body instantaneously, not through physics simulation.
    """
    pass

def resetJointStatesMultiDof(objectUniqueId, jointIndices, targetValues, targetVelocities=0, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    resetJointStatesMultiDof(objectUniqueId, jointIndices, targetValues, targetVelocities=0, physicsClientId=0)
    Reset the states (position, velocity etc) for multiple joints on a body instantaneously, not through physics simulation.
    """
    pass

def resetMeshData(*args, **kwargs): # real signature unknown
    """ Reset mesh data. Only implemented for deformable bodies. """
    pass

def resetSimulation(physicsClientId=0): # real signature unknown; restored from __doc__
    """
    resetSimulation(physicsClientId=0)
    Reset the simulation: remove all objects and start from an empty world.
    """
    pass

def resetVisualShapeData(*args, **kwargs): # real signature unknown
    """ Obsolete method, kept for backward compatibility, use changeVisualShapeData instead. """
    pass

def restoreState(*args, **kwargs): # real signature unknown
    """ Restore the full state of an existing world. """
    pass

def rotateVector(*args, **kwargs): # real signature unknown
    """ Rotate a vector using a quaternion. """
    pass

def saveBullet(*args, **kwargs): # real signature unknown
    """ Save the full state of the world to a .bullet file. """
    pass

def saveState(*args, **kwargs): # real signature unknown
    """ Save the full state of the world to memory. """
    pass

def saveWorld(filename): # real signature unknown; restored from __doc__
    """ Save a approximate Python file to reproduce the current state of the world: saveWorld(filename). (very preliminary and approximately) """
    pass

def setAdditionalSearchPath(*args, **kwargs): # real signature unknown
    """ Set an additional search path, used to load URDF/SDF files. """
    pass

def setCollisionFilterGroupMask(*args, **kwargs): # real signature unknown
    """ Set the collision filter group and the mask for a body. """
    pass

def setCollisionFilterPair(*args, **kwargs): # real signature unknown
    """ Enable or disable collision detection between two object links.Input are two object unique ids and two link indices and an enumto enable or disable collisions. """
    pass

def setDebugObjectColor(*args, **kwargs): # real signature unknown
    """ Override the wireframe debug drawing color for a particular object unique id / link index.If you ommit the color, the custom color will be removed. """
    pass

def setDefaultContactERP(defaultContactERP, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    setDefaultContactERP(defaultContactERP, physicsClientId=0)
    Set the amount of contact penetration Error Recovery Paramater (ERP) in each time step. 		This is an tuning parameter to control resting contact stability. This value depends on the time step.
    """
    pass

def setGravity(gravX, gravY, gravZ, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    setGravity(gravX, gravY, gravZ, physicsClientId=0)
    Set the gravity acceleration (x,y,z).
    """
    pass

def setInternalSimFlags(*args, **kwargs): # real signature unknown
    """ This is for experimental purposes, use at own risk, magic may or not happen """
    pass

def setJointMotorControl(*args, **kwargs): # real signature unknown
    """ This (obsolete) method cannot select non-zero physicsClientId, use setJointMotorControl2 instead.Set a single joint motor control mode and desired target value. There is no immediate state change, stepSimulation will process the motors. """
    pass

def setJointMotorControl2(*args, **kwargs): # real signature unknown
    """ Set a single joint motor control mode and desired target value. There is no immediate state change, stepSimulation will process the motors. """
    pass

def setJointMotorControlArray(*args, **kwargs): # real signature unknown
    """ Set an array of motors control mode and desired target value. There is no immediate state change, stepSimulation will process the motors.This is similar to setJointMotorControl2, with jointIndices as a list, and optional targetPositions, targetVelocities, forces, kds and kps as listsUsing setJointMotorControlArray has the benefit of lower calling overhead. """
    pass

def setJointMotorControlMultiDof(*args, **kwargs): # real signature unknown
    """ Set a single joint motor control mode and desired target value. There is no immediate state change, stepSimulation will process the motors.This method sets multi-degree-of-freedom motor such as the spherical joint motor. """
    pass

def setJointMotorControlMultiDofArray(*args, **kwargs): # real signature unknown
    """ Set control mode and desired target values for multiple motors. There is no immediate state change, stepSimulation will process the motors.This method sets multi-degree-of-freedom motor such as the spherical joint motor. """
    pass

def setPhysicsEngineParameter(*args, **kwargs): # real signature unknown
    """ Set some internal physics engine parameter, such as cfm or erp etc. """
    pass

def setRealTimeSimulation(enableRealTimeSimulation, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    setRealTimeSimulation(enableRealTimeSimulation, physicsClientId=0)
    Enable or disable real time simulation (using the real time clock, RTC) in the physics server. Expects one integer argument, 0 or 1
    """
    pass

def setTimeOut(*args, **kwargs): # real signature unknown
    """ Set the timeOut in seconds, used for most of the API calls. """
    pass

def setTimeStep(timestep, physicsClientId=0): # real signature unknown; restored from __doc__
    """
    setTimeStep(timestep, physicsClientId=0)
    Set the amount of time to proceed at each call to stepSimulation. (unit is seconds, typically range is 0.01 or 0.001)
    """
    pass

def setVRCameraState(*args, **kwargs): # real signature unknown
    """ Set properties of the VR Camera such as its root transform for teleporting or to track objects (camera inside a vehicle for example). """
    pass

def startStateLogging(*args, **kwargs): # real signature unknown
    """ Start logging of state, such as robot base position, orientation, joint positions etc. Specify loggingType (STATE_LOGGING_MINITAUR, STATE_LOGGING_GENERIC_ROBOT, STATE_LOGGING_VR_CONTROLLERS, STATE_LOGGING_CONTACT_POINTS, etc), fileName, optional objectUniqueId, maxLogDof, bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB. Function returns int loggingUniqueId """
    pass

def stepSimulation(physicsClientId=0): # real signature unknown; restored from __doc__
    """
    stepSimulation(physicsClientId=0)
    Step the simulation using forward dynamics.
    """
    pass

def stopStateLogging(*args, **kwargs): # real signature unknown
    """ Stop logging of robot state, given a loggingUniqueId. """
    pass

def submitProfileTiming(*args, **kwargs): # real signature unknown
    """ Add a custom profile timing that will be visible in performance profile recordings on the physics server.On the physics server (in GUI and VR mode) you can press 'p' to start and/or stop profile recordings """
    pass

def syncBodyInfo(physicsClientId=0): # real signature unknown; restored from __doc__
    """
    syncBodyInfo(physicsClientId=0)
    Update body and constraint/joint information, in case other clients made changes.
    """
    pass

def syncUserData(bodyUniqueIds=[], physicsClientId=0): # real signature unknown; restored from __doc__
    """
    syncUserData(bodyUniqueIds=[], physicsClientId=0)
    Update user data, in case other clients made changes.
    """
    pass

def unloadPlugin(*args, **kwargs): # real signature unknown
    """ Unload a plugin, given the pluginUniqueId. """
    pass

def unsupportedChangeScaling(*args, **kwargs): # real signature unknown
    """ Change the scaling of the base of an object.Warning: unsupported rudimentary feature that has many limitations. """
    pass

def vhacd(*args, **kwargs): # real signature unknown
    """ Compute volume hierarchical convex decomposition of an OBJ file. """
    pass

# classes

class error(Exception):
    # no doc
    def __init__(self, *args, **kwargs): # real signature unknown
        pass

    __weakref__ = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default
    """list of weak references to the object (if defined)"""



# variables with complex values

__loader__ = None # (!) real value is '<_frozen_importlib_external.ExtensionFileLoader object at 0x7f63143c5480>'

__spec__ = None # (!) real value is "ModuleSpec(name='pybullet', loader=<_frozen_importlib_external.ExtensionFileLoader object at 0x7f63143c5480>, origin='/home/quintin/Programming/snake-sim/venv/lib/python3.10/site-packages/pybullet.cpython-310-x86_64-linux-gnu.so')"

