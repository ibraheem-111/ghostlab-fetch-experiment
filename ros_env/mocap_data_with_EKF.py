import rospy
import csv
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from numpy.linalg import inv
import time

class LiveEKFNode:
    def __init__(self):
        rospy.init_node('ekf_pose_collector', anonymous=True)
        
        # --- EKF CONFIGURATION ---
        # State: [x, y, theta, v, omega]
        self.X = np.zeros(5) 
        
        # Covariance Matrix P (Initial uncertainty)
        self.P = np.diag([0.1**2, 0.1**2, (10*np.pi/180)**2, 1.0**2, (1.0)**2])
        
        # Measurement Noise R (Trust in Optitrack: High trust = low numbers)
        self.R = np.diag([0.02**2, 0.02**2, (1*np.pi/180)**2])
        
        # Process Noise Q (Trust in Physics/Prediction: Allow for some jitter)
        # We scale this by dt in the loop, but these are the base variances
        self.q_v = 0.5**2
        self.q_w = (10*np.pi/180)**2
        
        # Measurement Matrix H (We measure x, y, theta directly)
        self.H = np.zeros((3,5))
        self.H[0,0] = 1.0 # x
        self.H[1,1] = 1.0 # y
        self.H[2,2] = 1.0 # theta

        # Timing variables
        self.last_time = None
        
        # Data storage
        self.data_list = []
        self.csv_filename = "ekf_live_data_" + str(time.time()) + ".csv"
        
        # Subscribers
        self.topic = "/vrpn_client_node/fetch/pose"
        rospy.Subscriber(self.topic, PoseStamped, self.callback)
        rospy.on_shutdown(self.save_to_csv)
        
        rospy.loginfo(f"EKF Observer started on {self.topic}")
        rospy.spin()

    def angwrap(self, a):
        """Wrap angle to [-pi, pi)"""
        return (a + np.pi) % (2*np.pi) - np.pi

    def callback(self, msg):
        # 1. EXTRACT DATA
        stamp = msg.header.stamp
        current_time = stamp.to_sec() # Use float time!
        
        pose = msg.pose
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # Optitrack usually puts 'yaw' in the 3rd component (gamma), but verify your frame!
        meas_x = pose.position.x
        meas_y = pose.position.y
        meas_theta = euler[2] 

        # 2. HANDLE FIRST FRAME & DT
        if self.last_time is None:
            self.last_time = current_time
            self.X[0] = meas_x
            self.X[1] = meas_y
            self.X[2] = meas_theta
            return # Wait for second frame to calculate dt

        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0.0001: return # Avoid divide by zero or duplicate messages

        # --- EKF STEP 1: PREDICTION (Physics) ---
        x, y, th, v, w = self.X
        
        # Nonlinear motion model
        x_pred  = x + v * np.cos(th) * dt
        y_pred  = y + v * np.sin(th) * dt
        th_pred = self.angwrap(th + w * dt)
        
        # Jacobian F (Linearization of motion)
        F = np.eye(5)
        F[0,2] = -v * np.sin(th) * dt
        F[0,3] =  np.cos(th) * dt
        F[1,2] =  v * np.cos(th) * dt
        F[1,3] =  np.sin(th) * dt
        F[2,4] =  dt
        
        # Process Noise scaling
        Q = np.zeros((5,5))
        Q[3,3] = self.q_v * dt
        Q[4,4] = self.q_w * dt
        
        P_pred = F @ self.P @ F.T + Q
        
        # --- EKF STEP 2: UPDATE (Correction) ---
        z = np.array([meas_x, meas_y, meas_theta])
        hx = np.array([x_pred, y_pred, th_pred])
        
        y_tilde = z - hx
        y_tilde[2] = self.angwrap(y_tilde[2]) # Wrap angle residual
        
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ inv(S)
        
        self.X = np.array([x_pred, y_pred, th_pred, v, w]) + K @ y_tilde
        self.X[2] = self.angwrap(self.X[2])
        self.P = (np.eye(5) - K @ self.H) @ P_pred

        # 3. LOGGING & STORAGE
        # Get the "Hidden States"
        est_v = self.X[3]
        est_omega = self.X[4]

        rospy.loginfo(f"Live State -> V: {est_v:.3f} m/s | Omega: {est_omega:.3f} rad/s")

        data_entry = {
            "t": current_time,
            "x_raw": meas_x,
            "y_raw": meas_y,
            "theta_raw": meas_theta,
            "x_est": self.X[0],
            "y_est": self.X[1],
            "theta_est": self.X[2],
            "v_est": est_v,       # The hidden state!
            "omega_est": est_omega # The hidden state!
        }
        self.data_list.append(data_entry)

    def save_to_csv(self):
        rospy.loginfo("Saving EKF data to CSV...")
        with open(self.csv_filename, "w") as f:
            fieldnames = ["t", "x_raw", "y_raw", "theta_raw", "x_est", "y_est", "theta_est", "v_est", "omega_est"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.data_list)
        rospy.loginfo(f"Saved {len(self.data_list)} entries to {self.csv_filename}")

if __name__ == '__main__':
    try:
        LiveEKFNode()
    except rospy.ROSInterruptException:
        pass