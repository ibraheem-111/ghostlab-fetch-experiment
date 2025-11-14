#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import rospy
import csv
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
import time as pytime  # avoid shadowing 'time' with variables

class ConstantVelocityEKF(object):
    """
    3D constant-velocity EKF:
      state x = [px, py, pz, vx, vy, vz]^T
      measurement z = [px, py, pz]^T
    """
    def __init__(self, q_acc=0.5, r_pos=0.01):
        # Process noise (continuous white-noise acceleration, std^2)
        self.q_acc = float(q_acc)
        # Measurement noise variance (position)
        self.r_pos = float(r_pos)

        # State vector and covariance
        self.x = None  # will init on first measurement
        self.P = None

    def _make_F(self, dt):
        F = np.eye(6)
        F[0,3] = dt
        F[1,4] = dt
        F[2,5] = dt
        return F

    def _make_Q(self, dt):
        # Discretized Q for constant-velocity with white-noise acceleration
        dt2 = dt*dt
        dt3 = dt2*dt
        q = self.q_acc
        Q_pos = (dt3/3.0) * q
        Q_cross = (dt2/2.0) * q
        Q_vel = dt * q
        Q = np.zeros((6,6))
        for i in range(3):
            Q[i,i] = Q_pos
            Q[i, i+3] = Q_cross
            Q[i+3, i] = Q_cross
            Q[i+3, i+3] = Q_vel
        return Q

    def predict(self, dt):
        if self.x is None:
            return
        F = self._make_F(dt)
        Q = self._make_Q(dt)
        self.x = np.dot(F, self.x)
        self.P = np.dot(F, np.dot(self.P, F.T)) + Q

    def update(self, z):
        # z = [px, py, pz]
        H = np.zeros((3,6))
        H[0,0] = H[1,1] = H[2,2] = 1.0
        R = np.eye(3) * self.r_pos

        y = z - np.dot(H, self.x)                   # innovation
        S = np.dot(H, np.dot(self.P, H.T)) + R      # innovation cov
        K = np.dot(self.P, np.dot(H.T, np.linalg.inv(S)))  # Kalman gain

        self.x = self.x + np.dot(K, y)
        I_KH = np.eye(6) - np.dot(K, H)
        self.P = np.dot(I_KH, self.P)

    def initialize(self, z0, vel0=None):
        self.x = np.zeros((6,1))
        self.x[0,0] = z0[0]
        self.x[1,0] = z0[1]
        self.x[2,0] = z0[2]
        if vel0 is not None:
            self.x[3,0] = vel0[0]
            self.x[4,0] = vel0[1]
            self.x[5,0] = vel0[2]
        # Reasonable initial uncertainty: confident in position, unsure in velocity
        self.P = np.diag([0.05, 0.05, 0.05, 1.0, 1.0, 1.0])

class PoseEKFNode(object):
    def __init__(self):
        rospy.init_node('pose_ekf', anonymous=True)

        # Params (tune as needed; can also set via rosparam)
        self.ros_topic = rospy.get_param('~input_topic', '/vrpn_client_node/RigidBody004/pose')
        self.out_pose_topic = rospy.get_param('~out_pose_topic', '/pose_filtered')
        self.out_twist_topic = rospy.get_param('~out_twist_topic', '/velocity_ekf')
        q_acc = rospy.get_param('~q_acc', 0.5)       # m/s^2 process noise intensity
        r_pos = rospy.get_param('~r_pos', 0.01)      # m^2 measurement noise for position
        self.min_dt = rospy.get_param('~min_dt', 1e-4)

        self.ekf = ConstantVelocityEKF(q_acc=q_acc, r_pos=r_pos)
        self.last_t = None

        self.pose_pub = rospy.Publisher(self.out_pose_topic, PoseStamped, queue_size=20)
        self.twist_pub = rospy.Publisher(self.out_twist_topic, TwistStamped, queue_size=20)

        self.data_list = []

        rospy.Subscriber(self.ros_topic, PoseStamped, self.callback)
        rospy.on_shutdown(self.save_to_csv)
        rospy.loginfo("Listening to %s ... Ctrl+C to stop.", self.ros_topic)

    def callback(self, msg):
        # Time as float seconds (high resolution)
        t = msg.header.stamp.to_sec()

        # Measurement (raw position)
        z = np.array([[msg.pose.position.x],
                      [msg.pose.position.y],
                      [msg.pose.position.z]])

        # Initialize EKF on first message
        if self.ekf.x is None:
            self.ekf.initialize(z0=z.flatten(), vel0=[0.0, 0.0, 0.0])
            self.last_t = t
            self.publish_outputs(msg, self.ekf.x)  # publish zeros for vel initially
            self.log_row(t, msg, self.ekf.x)
            return

        # Delta time
        dt = t - self.last_t if self.last_t is not None else 0.0
        if dt < self.min_dt:
            # Skip predict/update if dt too small or time went backwards
            return

        # EKF predict -> update
        self.ekf.predict(dt)
        self.ekf.update(z)

        self.last_t = t

        # Publish filtered pose & velocity
        self.publish_outputs(msg, self.ekf.x)
        self.log_row(t, msg, self.ekf.x)

    def publish_outputs(self, msg, x):
        # x: [px, py, pz, vx, vy, vz]^T
        # Pose: filtered position + pass-through orientation (you can add orientation filtering later)
        p_out = PoseStamped()
        p_out.header = msg.header  # preserve timestamp/frame
        p_out.pose.position.x = float(x[0,0])
        p_out.pose.position.y = float(x[1,0])
        p_out.pose.position.z = float(x[2,0])
        p_out.pose.orientation = msg.pose.orientation
        self.pose_pub.publish(p_out)

        # Twist: EKF linear velocity
        tw = TwistStamped()
        tw.header = msg.header
        tw.twist.linear.x = float(x[3,0])
        tw.twist.linear.y = float(x[4,0])
        tw.twist.linear.z = float(x[5,0])
        # (angular velocity not estimated here)
        self.twist_pub.publish(tw)

    def log_row(self, t, msg, x):
        # For convenience: also log Euler angles (same as in your original script)
        q = (msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z,
             msg.pose.orientation.w)
        alpha, beta, gamma = tf.transformations.euler_from_quaternion(q)

        row = {
            "time": t,
            "x_raw": msg.pose.position.x,
            "y_raw": msg.pose.position.y,
            "z_raw": msg.pose.position.z,
            "x_f": float(x[0,0]),
            "y_f": float(x[1,0]),
            "z_f": float(x[2,0]),
            "vx": float(x[3,0]),
            "vy": float(x[4,0]),
            "vz": float(x[5,0]),
            "alpha": alpha,
            "beta": beta,
            "gamma": gamma
        }
        self.data_list.append(row)

    def save_to_csv(self):
        if not self.data_list:
            rospy.loginfo("No data to save.")
            return
        rospy.loginfo("Saving data to CSV...")
        file_name = "pose_ekf_50Hz_{:.3f}.csv".format(pytime.time())
        # Fixed CSV headers:
        headers = ["time","x_raw","y_raw","z_raw","x_f","y_f","z_f","vx","vy","vz","alpha","beta","gamma"]
        try:
            with open(file_name, "w") as f:
                writer = csv.DictWriter(f, fieldnames=headers)
                writer.writeheader()
                writer.writerows(self.data_list)
            rospy.loginfo("Saved %d entries to %s", len(self.data_list), file_name)
        except Exception as e:
            rospy.logerr("Failed to write CSV: %s", str(e))
        rospy.loginfo("Exiting gracefully.")

def main():
    node = PoseEKFNode()
    rospy.spin()

if __name__ == '__main__':
    main()
