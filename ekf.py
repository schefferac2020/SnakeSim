import numpy as np
from manifpy import SO3, SO3Tangent, SE3, SE3Tangent
from numpy.typing import NDArray
from utils import ang_vel_wedge, forward_kinematics

class EKF:
    # from (our head)
    class State:
        a: NDArray  # World frame acceleration
        q: NDArray  # World frame orientation
        w: NDArray  # World frame angular velocityes

        def __init__(self, a=np.array([0, 0, 0]), q = np.array([1, 0, 0, 0]), w=np.array([0, 0, 0])):
            self.a = a
            self.q = q
            self.w = w

    class Measurement:
        encoders: NDArray
        accelerations: NDArray
        gyroscope: NDArray

    def __init__(self, num_joints: int, link_length: float ) -> None:
        self.N = num_joints
        self.l = link_length
        self.state = self.State()
        self.P = np.eye(self.N) * 1e-6
        self.Q = np.eye(self.N) * 1e-6
        
        # self.state.q = np.array([[1, 0, 0, 0]]).T #qw qx qy qz

        self.tau = 25 # Acceleration damping

        self.g = np.array([0, 0, -9.81])

        self.VC_to_Body = np.eye(4)
    
    def set_VC_Transform(self, VC_to_Head_in):
        self.VC_to_Head = VC_to_Head_in

    def process_model(self, dt: float):
        '''Update the the state to be state_pred.
        
            @param dt: float    Timestep
        '''
        omega = ang_vel_wedge(self.state.w) 
        w_mag = np.linalg.norm(self.state.w)
        temp = w_mag * dt / 2

        q_pred = (np.cos(temp)*np.eye(4) + (2 / w_mag) * np.sin(temp) * omega) @ self.state.q
        w_pred = self.state.w # Constant angular velocity model
        a_pred = np.exp(self.tau*dt)*self.state.a

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
        t9 = 1.0/t8
        t10 = np.sqrt(t8)
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


    def predict(self, u: NDArray, dt: float) -> None:
        """
        :param u:   Commanded joint velocities
        :param dt:  Time step
        """

        # predict state with process model(self.state, u, dt)
        self.state = self.process_model(u, dt)
        F = self.process_jacobian(dt)
        # T state dynamics
        self.P = F @ self.P @ F.T + self.Q



    def tempy(self, i: int, q_in: NDArray):
        from sympy.abc import w, x, y, z
        from sympy import Matrix

        q = [w, x, y, z]

        R_q = Matrix([[w**2 + x**2 - y**2 - z**2,    2*(x * y - w * z),         2 * (x * z + w * y)],
                     [2 * (x * y + w * z),      w**2 - x**2 + y**2 - z**2,     2 * (y * z - w * x)],
                     [2 * (x * z - w * z),      2 * (y * z + w * x),       w**2 - x**2 - y**2 + z**2]])
        
        zippy = list(zip(q, q_in))

        result = R_q.diff(q[i])

        return np.array(result.subs(zippy))


    def update(self, m: Measurement) -> None:
        """
        :param m:   Measurement
        """
        predicted_accelerations = self.acceleration_prediction()
        predicted_gyroscope = self.gyroscope_prediction()
        
        m_vec = np.concatenate([m.accelerations, m.gyroscope])
        pred_vec = np.concatenate([predicted_accelerations, predicted_gyroscope]) 
        H = ... # TODO: AYYYYYYYYYYYYYYYYYYYYYYYYYYY DO THIS

        innovation = m_vec - pred_vec
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ innovation
        self.P = (np.eye(self.N) - K @ H) @ self.P

    def acceleration_prediction(self) -> NDArray:
        accel = np.zeros(3 * self.N)
        for i in range(self.N):

            # predicted acceleration due to gravity
            link_to_head = self.forward_kinematics(i, self.state.theta).rotation()
            body_to_head = self.VC_to_Head # virtual chassis frame transform
            link_to_body = body_to_head.T @ link_to_head

            normalized = self.state.q / np.linalg.norm(self.state.q)
            body_to_world = SO3(self.state.q).transform()
    
            # predicted acceleration due to internal motion
            accel_internal = ... # TODO
            
            # predicted acceleration from robot motion
            accel_robot = world_to_link @ self.state.a
            accel[3*i:3*(i+1)] = accel_g + accel_internal + accel_robot
        return accel
    
    def gyroscope_prediction(self) -> NDArray:
        gyro = np.zeros(3 * self.N)
        for i in range(self.N):
            link_to_head = self.forward_kinematics(i, self.state.theta)
            head_to_body = ... # virtual chassis frame transform
            link_to_body = head_to_body @ link_to_head
            dt = ... # TODO: get time step
            theta_prev = ... # TODO: get previous joint angles
            link_to_head_prev = self.forward_kinematics(i, theta_prev)
            head_to_body_prev = ... # virtual chassis frame transform
            link_to_body_prev = head_to_body_prev @ link_to_head_prev
            delta_R = link_to_body @ link_to_body_prev.T / dt

            # extract angular velocity from skew components
            w_z = delta_R[1, 0]
            w_y = delta_R[0, 2]
            w_x = delta_R[2, 1]
            gyro_internal = np.array([w_x, w_y, w_z])

            gyro[3*i:3*(i+1)] = gyro_internal + link_to_body.T @ self.state.w
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
    test_ekf.tempy(1, [1, 0, 0, 0])

