import numpy as np
import pybullet as p
from manifpy import SE3, SE3Tangent, SO3
from numpy.typing import NDArray


def to_SE3(position, orientation) -> NDArray:
    """Covert position and orientation to SE3 matrix
    
    Args: 
        position (np.array): position of frame 2 relative to frame 1
        orientation (np.array): xyzw quaternion representing orientation of frame 2 relative to frame 1
    """
    rotation_matrix = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)
    frame2_to_frame1 = np.eye(4)
    frame2_to_frame1[0:3, 0:3] = rotation_matrix
    frame2_to_frame1[:3, 3] = np.array(position)

    return frame2_to_frame1


def draw_frame(client, debug_items, frame_name, T_wrt_world):
    """
    Draws a frame in the world

    Args:
        client: pybullet client
        debug_items: dictionary representing debug items
        frame_name (str): Specifies the name/ID of the frame to create/update
        T_wrt_world (np.array): 4x4 transformation matrix that specified the frame wrt the world frame
    """
    line_width = 2
    line_length = 0.3
    # Draw axes for orientation
    position = (T_wrt_world @ np.array([[0], [0], [0], [1]])).reshape(-1)[0:3]

    x_axis_end = (T_wrt_world @ (np.array([[line_length], [0], [0], [1]]))).reshape(-1)[0:3]
    y_axis_end = (T_wrt_world @ (np.array([[0], [line_length], [0], [1]]))).reshape(-1)[0:3]
    z_axis_end = (T_wrt_world @ (np.array([[0], [0], [line_length], [1]]))).reshape(-1)[0:3]

    if frame_name not in debug_items.keys():
        debug_items[frame_name] = []
        debug_items[frame_name].append(client.addUserDebugLine(position, x_axis_end, [1, 0, 0], lineWidth=line_width))
        debug_items[frame_name].append(client.addUserDebugLine(position, y_axis_end, [0, 1, 0], lineWidth=line_width))
        debug_items[frame_name].append(client.addUserDebugLine(position, z_axis_end, [0, 0, 1], lineWidth=line_width))
    else:
        # Just update the current axes
        x_line_id = debug_items[frame_name][0]
        y_line_id = debug_items[frame_name][1]
        z_line_id = debug_items[frame_name][2]
        debug_items[frame_name][0] = client.addUserDebugLine(position, x_axis_end, [1, 0, 0], replaceItemUniqueId=x_line_id, lineWidth=line_width)
        debug_items[frame_name][1] = client.addUserDebugLine(position, y_axis_end, [0, 1, 0], replaceItemUniqueId=y_line_id, lineWidth=line_width)
        debug_items[frame_name][2] = client.addUserDebugLine(position, z_axis_end, [0, 0, 1], replaceItemUniqueId=z_line_id, lineWidth=line_width)


def remove_all_debug_items(client, debug_items):
    """ Removes all debug frames and boxes

    Args:
        client: pybullet client
        debug_items: dictionary representing debug items
    """
    for key in debug_items.keys():
        while len(debug_items[key]) > 0:
            client.removeUserDebugItem(debug_items[0])
            debug_items.pop(0)


def forward_kinematics(i: int, link_length: float, theta: NDArray) -> SE3:
    """
    :param i:       link index
    :param theta:   Joint angles
    :return:        Transformation matrix from joint i to body frame
    """
    g_si0 = SE3(0, link_length * (i), 0, 0, 0, 0)

    g_si = g_si0
    for j in reversed(range(i)):
        xi_even = SE3Tangent(np.array([0, 0, -link_length * (j + 1 / 2), 1, 0, 0]))
        xi_odd = SE3Tangent(np.array([link_length * (j + 1 / 2), 0, 0, 0, 0, 1]))
        if j % 2 == 1:
            g_si = (xi_even * theta[j]).exp() * g_si
        else:
            g_si = (xi_odd * theta[j]).exp() * g_si
    return g_si


def ang_vel_wedge(w) -> NDArray:
    """Wedge operator for angular velocity using skew symmetric matrix"""
    return np.array([[0, -w[0], -w[1], -w[2]],
                     [w[0], 0, w[2], -w[1]],
                     [w[1], -w[2], 0, w[0]],
                     [w[2], w[1], -w[0], 0]])


def make_so3_nonstupid(q) -> SO3:
    nq = q.copy()
    nq /= np.linalg.norm(nq)
    return SO3(nq)

def wxyz_too_xyzw(q):
    q_new = np.roll(q, -1)
    return q_new/np.linalg.norm(q_new)

def xyzw_too_wxyz(q):
    q_new = np.roll(q, 1)
    return q_new/np.linalg.norm(q_new)
