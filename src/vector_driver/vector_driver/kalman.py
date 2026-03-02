import numpy as np
import math
from vector_driver.utils import wrap_angle_pi

class KalmanFilter:

    def __init__(self):

        # self.P_last = np.diag([2.0, 2.0, np.deg2rad(15.0)])**2  # slightly more uncertainty to allow for corrections

        self.P_last = np.diag([
            0.001**2,  # 1 mm² variance in X (std dev = 1 mm)
            0.001*2,  # 1 mm² in Y
            np.deg2rad(2.0)**2  # orientation known to within 0.5°
        ])


        # self.Q = np.diag([2.0, 2.0, np.deg2rad(10.0)**2])   # process noise
        # self.R = np.diag([2.0, 2.0, np.deg2rad(10.0)**2])        # measurement noise

        # self.Q = np.diag([0.5, 0.5, np.deg2rad(0.5)**2])   # Trust odometry more
        self.Q = np.diag([0.004, 0.004, np.deg2rad(8.0)**2])

        self.R = np.diag([0.001, 0.001, np.deg2rad(2.0)**2])  # Camera is jumpy



        # self.Q = np.diag([
        #     2.0, 2.0, np.deg2rad(10.0)**2  # slightly more process noise on orientation
        # ])

        self.H = np.eye(3)

        self.x_last = 0.0
        self.y_last = 0.0
        self.theta_last = math.pi / 2

        self.mahalanobis_dist = None



    def initial_predict(self, velocity, timestep, theta):
        
        # ODOMETRY PREDICTOIN -- using the Motion Model Taken From: https://www.youtube.com/watch?v=LrsTBWf6Wsc
        
        theta_k = self.theta_last + theta * timestep
        theta_k = wrap_angle_pi(theta_k)

        x_k = self.x_last + velocity * math.cos(theta_k) * timestep
        y_k = self.y_last + velocity * math.sin(theta_k) * timestep

        # COMPUTE JACOBIAN MATRIX 
        # Look at notebook on how I got this: Partial Derivate with Jacobian Matrix
        F_k = np.array([[1, 0, -velocity * math.sin(theta_k) * timestep],
                        [0, 1, -velocity * math.cos(theta_k) * timestep], 
                        [0, 0, 1]])

        # Predict New Covariance 
        self.P_last = F_k @ self.P_last @ F_k.T + self.Q


        # UPDATE
        self.x_last = x_k
        self.y_last = y_k
        self.theta_last = theta_k

        return x_k, y_k, theta_k



    def update(self, x, y, theta):
        
        # R = np.diag([2.0, 2.0, np.deg2rad(10.0)**2])
        

        x_est = np.array([self.x_last, self.y_last, self.theta_last])
        z_k = np.array([x , y, theta]) # camera measurement
        y_k = z_k - x_est # difference between measurement and prediction

        y_k[2] = wrap_angle_pi(y_k[2])

        # KALMAN GAIN
        S = self.H @ self.P_last @ self.H.T + self.R
        K = self.P_last @ self.H.T @ np.linalg.inv(S)
        # print("Kalman Gain K:\n", K)

        # CHATGPT Mahalanobis distance
        S_inv = np.linalg.inv(S)
        self.mahalanobis_dist = y_k.T @ S_inv @ y_k

        # Threshold can be tuned; for 3D, 95% confidence ~7.81 (chi-square)
        if self.mahalanobis_dist > 20:
            # Reject measurement as outlier, skip update
            # print(f"Measurement rejected: Mahalanobis dist = {self.mahalanobis_dist:.2f}")
            return self.x_last, self.y_last, self.theta_last, None
        else: 
            print("USING READING")

        # Update state
        x_updated = x_est + K @ y_k

        # print(f"Updated state before covariance update: {x_updated}")

        # Update Error
        I = np.eye(3)
        self.P_last = (I - K @ self.H) @ self.P_last


        # Save updated state
        self.x_last = x_updated[0]
        self.y_last = x_updated[1]
        self.theta_last = x_updated[2]
        self.theta_last = wrap_angle_pi(x_updated[2])


        return self.x_last, self.y_last, self.theta_last, z_k
