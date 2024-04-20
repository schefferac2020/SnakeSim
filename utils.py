import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
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


def R_to_q(R: NDArray):
    """Convert rotation matrix to quaternion"""

    q = np.zeros(4)
    q[0] = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    q[1] = (R[2, 1] - R[1, 2]) / (4 * q[0])
    q[2] = (R[0, 2] - R[2, 0]) / (4 * q[0])
    q[3] = (R[1, 0] - R[0, 1]) / (4 * q[0])

    q = q / np.linalg.norm(q)
    return q


def draw_line(debug_items, line_name, position, direction, color, line_width=2):
    if line_name not in debug_items.keys():
        debug_items[line_name] = p.addUserDebugLine(position, direction, color, lineWidth=line_width)
    else:
        # Just update the current axes
        replace_id = debug_items[line_name]

        debug_items[line_name] = p.addUserDebugLine(position, direction, color, replaceItemUniqueId=replace_id, lineWidth=line_width)


def draw_frame(debug_items, frame_name, T_wrt_world):
    """
    Draws a frame in the world

    Args:
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
        debug_items[frame_name].append(p.addUserDebugLine(position, x_axis_end, [1, 0, 0], lineWidth=line_width))
        debug_items[frame_name].append(p.addUserDebugLine(position, y_axis_end, [0, 1, 0], lineWidth=line_width))
        debug_items[frame_name].append(p.addUserDebugLine(position, z_axis_end, [0, 0, 1], lineWidth=line_width))
    else:
        # Just update the current axes
        x_line_id = debug_items[frame_name][0]
        y_line_id = debug_items[frame_name][1]
        z_line_id = debug_items[frame_name][2]
        debug_items[frame_name][0] = p.addUserDebugLine(position, x_axis_end, [1, 0, 0], replaceItemUniqueId=x_line_id, lineWidth=line_width)
        debug_items[frame_name][1] = p.addUserDebugLine(position, y_axis_end, [0, 1, 0], replaceItemUniqueId=y_line_id, lineWidth=line_width)
        debug_items[frame_name][2] = p.addUserDebugLine(position, z_axis_end, [0, 0, 1], replaceItemUniqueId=z_line_id, lineWidth=line_width)


def remove_all_debug_items(debug_items):
    """ Removes all debug frames and boxes

    Args:
        debug_items: dictionary representing debug items
    """
    for key in debug_items.keys():
        while len(debug_items[key]) > 0:
            p.removeUserDebugItem(debug_items[0])
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


def wxyz_to_xyzw(q):
    q_new = np.roll(q, -1)
    return q_new / np.linalg.norm(q_new)


def xyzw_too_wxyz(q):
    q_new = np.roll(q, 1)
    return q_new / np.linalg.norm(q_new)

def plot_accel(ts, accel_data, link_ids, fig_name):
    fig, ax = plt.subplots(3, 1, sharex=True)
    for i in link_ids:
        ax[0].plot(ts, accel_data[:, i*3], label=f"Link {i+1}")
    ax[0].grid(True)
    ax[0].set_title(fig_name + " X")
    ax[0].set_ylabel("m/s^2")
    ax[0].set_xlabel("Time (s)")
    ax[0].legend()
    
    for i in link_ids:
        ax[1].plot(ts, accel_data[:, i*3 + 1], label=f"Link {i+1}")
    ax[1].grid(True)
    ax[1].set_title(fig_name + " Y")
    ax[1].set_ylabel("m/s^2")
    ax[1].set_xlabel("Time (s)")
    ax[1].legend()
    
    for i in link_ids:
        ax[2].plot(ts, accel_data[:, i*3 + 2], label=f"Link {i+1}")
    ax[2].grid(True)
    ax[2].set_title(f"{fig_name} Z")
    ax[2].set_ylabel("m/s^2")
    ax[2].set_xlabel("Time (s)")
    ax[2].legend()
    # plt.show()

def plot_accel_combined(ts, accel_data, pred_accel_data, link_ids, fig_name):
    fig, ax = plt.subplots(3, 1, sharex=True)
    for i in link_ids:
        line, = ax[0].plot(ts, accel_data[:, i*3], label=f"Link {i+1}")
        ax[0].plot(ts, pred_accel_data[:, i*3], label=f"Predicted Link {i+1}", linestyle="--", color=line.get_color())
    ax[0].grid(True)
    ax[0].set_title(fig_name + " X")
    ax[0].set_ylabel("m/s^2")
    ax[0].set_xlabel("Time (s)")
    ax[0].legend()
    
    for i in link_ids:
        line, = ax[1].plot(ts, accel_data[:, i*3 + 1], label=f"Link {i+1}")
        ax[1].plot(ts, pred_accel_data[:, i*3 + 1], label=f"Predicted Link {i+1}", linestyle="--", color=line.get_color())
    ax[1].grid(True)
    ax[1].set_title(fig_name + " Y")
    ax[1].set_ylabel("m/s^2")
    ax[1].set_xlabel("Time (s)")
    ax[1].legend()
    
    for i in link_ids:
        line, = ax[2].plot(ts, accel_data[:, i*3 + 2], label=f"Link {i+1}")
        ax[2].plot(ts, pred_accel_data[:, i*3 + 2], label=f"Predicted Link {i+1}", linestyle="--", color=line.get_color())
    ax[2].grid(True)
    ax[2].set_title(f"{fig_name} Z")
    ax[2].set_ylabel("m/s^2")
    ax[2].set_xlabel("Time (s)")
    ax[2].legend()

def plot_gyro(ts, gyro_data, link_ids):
    fig, ax = plt.subplots(3, 1, sharex=True)
    for i in link_ids:
        ax[0].plot(ts, gyro_data[:, i*3], label=f"Link {i+1}")
    ax[0].grid(True)
    ax[0].set_title("Gyro X")
    ax[0].set_ylabel("rad/s")
    ax[0].set_xlabel("Time (s)")
    ax[0].legend()
    
    for i in link_ids:
        ax[1].plot(ts, gyro_data[:, i*3 + 1], label=f"Link {i+1}")
    ax[1].grid(True)
    ax[1].set_title("Gyro Y")
    ax[1].set_ylabel("rad/s")
    ax[1].set_xlabel("Time (s)")
    ax[1].legend()
    
    for i in link_ids:
        ax[2].plot(ts, gyro_data[:, i*3 + 2], label=f"Link {i+1}")
    ax[2].grid(True)
    ax[2].set_title("Gyro Z")
    ax[2].set_ylabel("rad/s")
    ax[2].set_xlabel("Time (s)")
    ax[2].legend()
    # plt.show()
    
def plot_vel(ts, vel_data, link_ids):
    fig, ax = plt.subplots(3, 1, sharex=True)
    for i in link_ids:
        ax[0].plot(ts, vel_data[:, i*3], label=f"Link {i+1}")
    ax[0].grid(True)
    ax[0].set_title("Velocity X")
    ax[0].set_ylabel("m/s")
    ax[0].set_xlabel("Time (s)")
    ax[0].legend()
    
    for i in link_ids:
        ax[1].plot(ts, vel_data[:, i*3 + 1], label=f"Link {i+1}")
    ax[1].grid(True)
    ax[1].set_title("Velocity Y")
    ax[1].set_ylabel("m/s")
    ax[1].set_xlabel("Time (s)")
    ax[1].legend()
    
    for i in link_ids:
        ax[2].plot(ts, vel_data[:, i*3 + 2], label=f"Link {i+1}")
    ax[2].grid(True)
    ax[2].set_title("Velocity Z")
    ax[2].set_ylabel("m/s")
    ax[2].set_xlabel("Time (s)")
    ax[2].legend()
    # plt.show()

def plot_joint_angles(ts, cmd_angle_data, enc_data, joint_ids):
    fig, ax = plt.subplots(len(joint_ids), 1, sharex=True)
    for i, joint_id in enumerate(joint_ids):
        ax[i].plot(ts, cmd_angle_data[:, joint_id], label=f"Commanded Angle")
        ax[i].plot(ts, enc_data[:, joint_id], label=f"Encoder Angle")
        ax[i].grid(True)
        ax[i].set_title(f"Joint {joint_id}")
        ax[i].set_ylabel("rad")
        ax[i].legend()
    ax[-1].set_xlabel("Time (s)")
    # plt.show()

def plot_ekf_data(t1, ekf_a_data, ekf_w_data, ekf_q_data, gt_q_data):
    
    # plot acceleration
    fig, ax = plt.subplots(3, 1, sharex=True)
    ax[0].plot(t1, ekf_a_data[:, 0])
    ax[0].grid(True)
    ax[0].set_title("Accel X")
    ax[0].set_ylabel("m/s^2")

    ax[1].plot(t1, ekf_a_data[:, 1])
    ax[1].grid(True)
    ax[1].set_title("Accel Y")
    ax[1].set_ylabel("m/s^2")
    
    ax[2].plot(t1, ekf_a_data[:, 2])
    ax[2].grid(True)
    ax[2].set_title("Accel Z")
    ax[2].set_ylabel("m/s^2")
    ax[2].set_xlabel("Time (s)")
    # plt.show()

    # plot angular velocity
    fig, ax = plt.subplots(3, 1, sharex=True)
    ax[0].plot(t1, ekf_w_data[:, 0])
    ax[0].grid(True)
    ax[0].set_title("Gyro X")
    ax[0].set_ylabel("rad/s")
    
    ax[1].plot(t1, ekf_w_data[:, 1])
    ax[1].grid(True)
    ax[1].set_title("Gyro Y")
    ax[1].set_ylabel("rad/s")
    
    ax[2].plot(t1, ekf_w_data[:, 2])
    ax[2].grid(True)
    ax[2].set_title("Gyro Z")
    ax[2].set_ylabel("rad/s")
    
    # plot quaternion
    fig, ax = plt.subplots(4, 1, sharex=True)
    ax[0].plot(t1, ekf_q_data[:, 0], label="EKF")
    ax[0].plot(t1, gt_q_data[:, 0], label="Ground Truth")
    ax[0].grid(True)
    ax[0].set_title("Quaternion W")
    ax[0].legend()
    
    ax[1].plot(t1, ekf_q_data[:, 1], label="EKF")
    ax[1].plot(t1, gt_q_data[:, 1], label="Ground Truth")
    ax[1].grid(True)
    ax[1].set_title("Quaternion X")
    
    ax[2].plot(t1, ekf_q_data[:, 2], label="EKF")
    ax[2].plot(t1, gt_q_data[:, 2], label="Ground Truth")
    ax[2].grid(True)
    ax[2].set_title("Quaternion Y")
    
    ax[3].plot(t1, ekf_q_data[:, 3], label="EKF")
    ax[3].plot(t1, gt_q_data[:, 3], label="Ground Truth")
    ax[3].grid(True)
    ax[3].set_title("Quaternion Z")
    ax[3].set_xlabel("Time (s)")