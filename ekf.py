from dataclasses import dataclass, field

import numpy as np
from manifpy import SO3, SO3Tangent, SE3, SE3Tangent
from numpy.typing import NDArray
from utils import ang_vel_wedge, forward_kinematics, make_so3_nonstupid, wxyz_to_xyzw, R_to_q

class EKF:
    # from (our head)
    @dataclass
    class State:
        a: NDArray = field(default_factory=lambda: np.zeros(3))
        q: NDArray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
        w: NDArray = field(default_factory=lambda: np.zeros(3))

        # def __init__(self, a=np.array([0.0, 0.0, 0.0]), q = np.array([1.0, 0.0, 0.0, 0.0]), w=np.array([0.0, 0.0, 0.0])):
        #     self.a = a
        #     self.q = q
        #     self.w = w

    def __init__(self, num_joints: int, link_length: float ) -> None:
        self.n_joints = num_joints
        self.n_links = num_joints + 1
        self.n_states = 10
        self.l = link_length
        self.state = self.State()
        self.P = np.eye(self.n_states) * 1e-1
        self.Q = np.eye(self.n_states) * 1e-2
        self.R = np.eye(6*self.n_links) * 1e-2
        self.thetas = np.zeros(self.n_joints)
        
        # self.state.q = np.array([[1.0, 0, 0, 0]]).T #qw qx qy qz
        self.T_head_to_world = np.eye(4)

        self.tau = 25 # Acceleration damping
        self.g = np.array([0, 0, -9.81])
        self.VC_to_head = SE3.Identity()
        self.precompute_derivatives()

        # initialize previous time steps for computing derivatives
        self.p_prev = np.array([self.forward_kinematics(i, self.thetas).translation() for i in range(self.n_links)]).T
        self.p_prev_prev = np.copy(self.p_prev)
        self.W_prev = np.array([SO3(self.forward_kinematics(i, self.thetas).quat()) for i in range(self.n_links)])
        
    
    def set_VC_Transform(self, VC_to_head_in):
        self.VC_to_head = SE3(position=VC_to_head_in[:3, 3], quaternion=R_to_q(VC_to_head_in[:3, :3]))

    def set_head_to_world_Transform(self, T_head_to_world_in):
        '''For debugging only'''
        self.T_head_to_world = T_head_to_world_in

    def process_model(self, dt: float):
        '''Update the the state to be state_pred.
        
            @param dt: float    Timestep
        '''
        omega = ang_vel_wedge(self.state.w) 
        w_mag = np.linalg.norm(self.state.w)

        q_pred = ...
        if w_mag != 0:
            temp = w_mag * dt / 2
            q_pred = (np.cos(temp)*np.eye(4) + (2 / w_mag) * np.sin(temp) * omega) @ self.state.q
        else:
            q_pred = self.state.q

        w_pred = self.state.w # Constant angular velocity model
        a_pred = np.exp(-self.tau*dt)*self.state.a

        #Update state
        self.state.q = q_pred
        self.state.w = w_pred
        self.state.a = a_pred
    
    def process_jacobian(self, dt):
        # Get them variables
        q1 = self.state.q[0]
        q2 = self.state.q[1]
        q3 = self.state.q[2]
        q4 = self.state.q[3]
        w1 = self.state.w[0]
        w2 = self.state.w[1]
        w3 = self.state.w[2]
        t2 = w1**2
        t3 = w2**2
        t4 = w3**2
        t5 = dt*self.tau
        t6 = -t5
        t8 = t2+t3+t4
        t7 = np.exp(t6)

        t9 = 0
        if t8 !=0:
            t9 = 1.0/t8

        t10 = np.sqrt(t8)

        t11 = 0
        if t10 != 0:
            t11 = 1.0/t10

        t13 = (dt*t10)/2.0
        t12 = t11**3
        t14 = np.cos(t13)
        t15 = np.sin(t13)
        t16 = q1*t11*t15*2.0
        t17 = q2*t11*t15*2.0
        t18 = q3*t11*t15*2.0
        t19 = q4*t11*t15*2.0
        t20 = t11*t15*w1*2.0
        t21 = t11*t15*w2*2.0
        t22 = t11*t15*w3*2.0
        t23 = dt*q1*t9*t14*w1*w2
        t24 = dt*q1*t9*t14*w1*w3
        t25 = dt*q2*t9*t14*w1*w2
        t26 = dt*q1*t9*t14*w2*w3
        t27 = dt*q2*t9*t14*w1*w3
        t28 = dt*q3*t9*t14*w1*w2
        t29 = dt*q2*t9*t14*w2*w3
        t30 = dt*q3*t9*t14*w1*w3
        t31 = dt*q4*t9*t14*w1*w2
        t32 = dt*q3*t9*t14*w2*w3
        t33 = dt*q4*t9*t14*w1*w3
        t34 = dt*q4*t9*t14*w2*w3
        t41 = q1*t12*t15*w1*w2*2.0
        t42 = q1*t12*t15*w1*w3*2.0
        t43 = q2*t12*t15*w1*w2*2.0
        t44 = q1*t12*t15*w2*w3*2.0
        t45 = q2*t12*t15*w1*w3*2.0
        t46 = q3*t12*t15*w1*w2*2.0
        t47 = q2*t12*t15*w2*w3*2.0
        t48 = q3*t12*t15*w1*w3*2.0
        t49 = q4*t12*t15*w1*w2*2.0
        t50 = q3*t12*t15*w2*w3*2.0
        t51 = q4*t12*t15*w1*w3*2.0
        t52 = q4*t12*t15*w2*w3*2.0
        t35 = -t17
        t36 = -t18
        t37 = -t19
        t38 = -t20
        t39 = -t21
        t40 = -t22
        t53 = -t27
        t54 = -t28
        t55 = -t34
        t56 = -t41
        t57 = -t42
        t58 = -t44

        mt1 = np.array([t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
               0.0,t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
               0.0,0.0,t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
               0.0,0.0,0.0,t14,t20,t21,t22,0.0,0.0,0.0,
               0.0,0.0,0.0,t38,t14,t40,t21,0.0,0.0,0.0,
               0.0,0.0,0.0,t39,t22,t14,t38,0.0,0.0,0.0,
               0.0,0.0,0.0,t40,t39,t20,t14,0.0,0.0,0.0,
               0.0,0.0,0.0, -t33+t35+t46+t51+t54+q2*t2*t12*t15*2.0-dt*q2*t2*t9*t14-(dt*q1*t11*t15*w1)/2.0, t16+t30-t31-t48+t49-q1*t2*t12*t15*2.0+dt*q1*t2*t9*t14-(dt*q2*t11*t15*w1)/2.0, t19+t23+t45+t53+t56-q4*t2*t12*t15*2.0+dt*q4*t2*t9*t14-(dt*q3*t11*t15*w1)/2.0])
        
        mt2 = np.array([t24+t25+t36-t43+t57+q3*t2*t12*t15*2.0-dt*q3*t2*t9*t14-(dt*q4*t11*t15*w1)/2.0,1.0,0.0,0.0,0.0,0.0,0.0,-t25+t36+t43+t52+t55+q3*t3*t12*t15*2.0-dt*q3*t3*t9*t14-(dt*q1*t11*t15*w2)/2.0,t23+t32+t37-t50+t56+q4*t3*t12*t15*2.0-dt*q4*t3*t9*t14-(dt*q2*t11*t15*w2)/2.0,t16-t29+t31+t47-t49-q1*t3*t12*t15*2.0+dt*q1*t3*t9*t14-(dt*q3*t11*t15*w2)/2.0,
               t17+t26+t46+t54+t58-q2*t3*t12*t15*2.0+dt*q2*t3*t9*t14-(dt*q4*t11*t15*w2)/2.0,0.0,1.0,0.0,0.0,0.0,0.0])
        
        
        mt3 = np.array([-t32+t37+t45+t50+t53+q4*t4*t12*t15*2.0-dt*q4*t4*t9*t14-(dt*q1*t11*t15*w3)/2.0, t18+t24+t52+t55+t57-q3*t4*t12*t15*2.0+dt*q3*t4*t9*t14-(dt*q2*t11*t15*w3)/2.0, t26+t33+t35-t51+t58+q2*t4*t12*t15*2.0-dt*q2*t4*t9*t14-(dt*q3*t11*t15*w3)/2.0,t16+t29-t30-t47+t48-q1*t4*t12*t15*2.0+dt*q1*t4*t9*t14-(dt*q4*t11*t15*w3)/2.0,0.0,0.0,1.0])
 
        jacobi = np.concatenate((mt1, mt2, mt3)).reshape(10, 10).T
        return jacobi

    def measurement_jacobian(self) -> NDArray:
        # TODO: Implement this
        H = np.zeros((6*self.n_links, self.n_states))
        for i in range(self.n_links):
            link_to_head = self.forward_kinematics(i, self.thetas)
            R_link_to_head = link_to_head.rotation()
            R_body_to_head = self.VC_to_head.rotation()
            link_to_body = R_body_to_head.T @ R_link_to_head

            daccel_da = link_to_body.T @ SO3(wxyz_to_xyzw(self.state.q)).rotation().T
            daccel_dqw = link_to_body.T @ self.dR_dq[0](self.state.q).T @ (self.g + self.state.a)
            daccel_dqx = link_to_body.T @ self.dR_dq[1](self.state.q).T @ (self.g + self.state.a)
            daccel_dqy = link_to_body.T @ self.dR_dq[2](self.state.q).T @ (self.g + self.state.a)
            daccel_dqz = link_to_body.T @ self.dR_dq[3](self.state.q).T @ (self.g + self.state.a)
            dgyro_dw = link_to_body.T
            
            H[3*i:3*(i+1), 0:3] = daccel_da
            H[3*i:3*(i+1), 3:7] = np.array([daccel_dqw, daccel_dqx, daccel_dqy, daccel_dqz]).T
            offset = self.n_links
            H[offset + 3*i:offset + 3*(i+1), 7:10] = dgyro_dw
        return H

    def predict(self, dt: float) -> None:
        self.process_model(dt)
        self.state.q /= np.linalg.norm(self.state.q)
        
        F = self.process_jacobian(dt)
        self.P = F @ self.P @ F.T + self.Q

    def precompute_derivatives(self):
        from sympy.abc import w, x, y, z
        from sympy import Matrix, lambdify

        q = [w, x, y, z]

        R_q = Matrix([[w**2 + x**2 - y**2 - z**2,    2*(x * y - w * z),         2 * (x * z + w * y)],
                     [2 * (x * y + w * z),      w**2 - x**2 + y**2 - z**2,     2 * (y * z - w * x)],
                     [2 * (x * z - w * z),      2 * (y * z + w * x),       w**2 - x**2 - y**2 + z**2]])

        self.dR_dq = []
        for i in range(4):
            expr = R_q.diff(q[i])
            self.dR_dq.append(lambdify([q], expr, "numpy"))
        
    def update(self, encoders, accelerations, gyros, dt) -> None:
        self.thetas = np.copy(encoders)

        predicted_accelerations = self.acceleration_prediction(dt)
        predicted_gyroscope = self.gyroscope_prediction(dt)

        m_vec = np.concatenate([accelerations, gyros])
        pred_vec = np.concatenate([predicted_accelerations, predicted_gyroscope]) 
        H = self.measurement_jacobian()

        innovation = m_vec - pred_vec
        # print(f"innovation: {innovation}")
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        state_vec = np.concatenate([self.state.a, self.state.q, self.state.w])
        state_vec = state_vec + K @ innovation
        self.state.a = state_vec[0:3]
        self.state.q = state_vec[3:7]
        self.state.w = state_vec[7:10]
        
        self.P = (np.eye(self.n_states) - K @ H) @ self.P

        # debug
        self.last_predicted_accelerations = predicted_accelerations
        self.last_predicted_gyroscope = predicted_gyroscope
        self.last_innovation = np.copy(innovation)
        
        # normalize quaternion to prevent divergence
        self.state.q /= np.linalg.norm(self.state.q)

    def acceleration_prediction(self, dt) -> NDArray:
        accel = np.zeros(3 * self.n_links)
        for i in range(self.n_links):

            # predicted acceleration due to gravity
            link_to_head = self.forward_kinematics(i, self.thetas)
            R_link_to_head = link_to_head.rotation()
            p_link_in_head = np.vstack((link_to_head.translation().reshape(3,-1), np.array([1])))

            R_body_to_head = self.VC_to_head.rotation()
            p_body_to_head = np.vstack((self.VC_to_head.translation().reshape(3,-1), np.array([1])))
            body_to_head = np.eye(4)
            body_to_head[:3, :3] = R_body_to_head
            body_to_head[:3, 3] = p_body_to_head[:3, 0]

            link_to_body = body_to_head[:3,:3].T @ link_to_head.rotation()


            p_link_in_body = (np.linalg.inv(body_to_head) @ p_link_in_head)[:3, 0]
            if i == 0:
                # print(p_link_in_body)
                ...

            # body_to_world_q = R_to_q(self.T_head_to_world[:3,:3] @ R_body_to_head)
            # body_to_world = SO3(wxyz_to_xyzw(body_to_world_q)).rotation()
            body_to_world = SO3(wxyz_to_xyzw(self.state.q)).rotation()

            # predicted acceleration due to internal motion
            link_accel_in_body = (p_link_in_body - 2*self.p_prev[:, i] + self.p_prev_prev[:, i]) / (dt**2) / 1000
            self.p_prev_prev[:, i] = self.p_prev[:, i]
            self.p_prev[:, i] = p_link_in_body

            # print(link_to_body.T.shape, body_to_world.T.shape, self.state.a.shape, R_body_to_head.T.shape, link_accel_in_head.shape)
            accel_pred = link_to_body.T @ body_to_world.T @ (self.g + self.state.a) + link_to_body.T @ link_accel_in_body# + R_body_to_head.T @ link_accel_in_head 
            # accel_pred = link_accel_in_body

            # predicted acceleration from robot motion
            accel[3*i:3*(i+1)] = accel_pred
        return accel
    
    def gyroscope_prediction(self, dt) -> NDArray:
        gyro = np.zeros(3 * self.n_links)
        for i in range(self.n_links):
            link_to_head = self.forward_kinematics(i, self.thetas)
            R_link_to_head = link_to_head.rotation()
            R_body_to_head = self.VC_to_head.rotation()
            link_to_body = R_body_to_head.T @ R_link_to_head
            
            W = SO3(self.VC_to_head.quat()).inverse() * SO3(link_to_head.quat())
            gyro_internal = self.W_prev[i].rminus(W).coeffs() / dt
            self.W_prev[i] = W
            
            gyro_pred = link_to_body.T @ self.state.w + gyro_internal

            gyro[3*i:3*(i+1)] = gyro_pred
        return gyro
    
    def forward_kinematics(self, i, thetas):
        return forward_kinematics(i, self.l, thetas)
    

if __name__ == '__main__':
    test_ekf = EKF(10, 0.5)
    test_ekf.state.a = np.array([1, 2, 3])
    test_ekf.state.w = np.array([1, 2.5, 0.5])
    test_ekf.state.q = np.array([0, 1, 0, 0])
    F = test_ekf.process_jacobian(0.1)
    # print(np.array_str(F, precision=4, suppress_small=True))
    test_ekf.dR_dq(1, [1, 0, 0, 0])

