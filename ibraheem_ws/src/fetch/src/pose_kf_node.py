#!/usr/bin/env python
"""
pose_kf_node.py (logging-only)

- Subscribes to robot & human PoseStamped
- Runs a simple linear constant-velocity KF per body
- Predicts at fixed timer rate, updates when measurements arrive
- Logs filtered rows into memory and saves CSVs on shutdown
- DOES NOT PUBLISH any ROS topics
"""

from __future__ import print_function
import rospy
import tf
import numpy as np
import csv
import time as _time
from geometry_msgs.msg import PoseStamped

# Input topics
ROBOT_TOPIC = "/vrpn_client_node/Fetch/pose"
HUMAN_TOPIC = "/vrpn_client_node/Human/pose"

# In-memory logs (saved on shutdown)
robot_log = []
human_log = []


class ConstantVelocityKF(object):
    """Linear KF: state = [x,y,theta,vx,vy,omega]^T; meas = [x,y,theta]."""

    def __init__(self, name,
                 q_pos=1e-4, q_theta=1e-4,
                 q_vel=1e-2, q_omega=1e-2,
                 r_pos=1e-4, r_theta=1e-3):
        self.name = name
        self.q_pos, self.q_theta = q_pos, q_theta
        self.q_vel, self.q_omega = q_vel, q_omega
        self.R = np.diag([r_pos, r_pos, r_theta])

        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.x = None
        self.P = None
        self.last_t = None

        # asynchronous measurement buffering
        self.z_latest = None   # (x,y,theta,roll,pitch,z)
        self.z_time = None
        self.has_new_measurement = False

    @staticmethod
    def _wrap_angle(a):
        return (a + np.pi) % (2.0 * np.pi) - np.pi

    def initialize(self, t, x_meas, y_meas, theta_meas):
        self.x = np.zeros((6, 1))
        self.x[0, 0] = x_meas
        self.x[1, 0] = y_meas
        self.x[2, 0] = self._wrap_angle(theta_meas)
        self.P = np.diag([1e-3, 1e-3, 1e-2, 1.0, 1.0, 1.0])
        self.last_t = t
        rospy.loginfo("%s: initialized at t=%.3f" % (self.name, t))

    def step_predict_only(self, t):
        if self.x is None:
            return
        dt = t - self.last_t if (self.last_t is not None) else 1e-3
        if dt <= 0.0:
            dt = 1e-3
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        Q = np.diag([
            self.q_pos * dt,
            self.q_pos * dt,
            self.q_theta * dt,
            self.q_vel * dt,
            self.q_vel * dt,
            self.q_omega * dt
        ])
        self.x = F.dot(self.x)
        self.P = F.dot(self.P).dot(F.T) + Q
        self.last_t = t

    def step_update_only(self, x_meas, y_meas, theta_meas):
        if self.x is None:
            tnow = self.z_time if self.z_time is not None else rospy.get_time()
            self.initialize(tnow, x_meas, y_meas, theta_meas)
            return

        z = np.array([[x_meas], [y_meas], [self._wrap_angle(theta_meas)]])
        z_pred = self.x[0:3]
        y = z - z_pred
        y[2, 0] = self._wrap_angle(y[2, 0])

        S = self.H.dot(self.P).dot(self.H.T) + self.R
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))

        self.x = self.x + K.dot(y)
        I = np.eye(6)
        self.P = (I - K.dot(self.H)).dot(self.P)
        self.x[2, 0] = self._wrap_angle(self.x[2, 0])

    def current_state(self):
        if self.x is None:
            return None
        return (float(self.x[0, 0]),
                float(self.x[1, 0]),
                float(self.x[2, 0]),
                float(self.x[3, 0]),
                float(self.x[4, 0]),
                float(self.x[5, 0]))


# Globals
robot_kf = None
human_kf = None


def pose_to_xytheta(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    q = msg.pose.orientation
    quat = (q.x, q.y, q.z, q.w)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
    t = msg.header.stamp.to_sec()
    return x, y, yaw, roll, pitch, z, t


def robot_callback(msg):
    global robot_kf
    x, y, yaw, roll, pitch, z, t = pose_to_xytheta(msg)
    robot_kf.z_latest = (x, y, yaw, roll, pitch, z)
    robot_kf.z_time = t
    robot_kf.has_new_measurement = True


def human_callback(msg):
    global human_kf
    x, y, yaw, roll, pitch, z, t = pose_to_xytheta(msg)
    human_kf.z_latest = (x, y, yaw, roll, pitch, z)
    human_kf.z_time = t
    human_kf.has_new_measurement = True


def timer_event(event):
    """Predict at fixed rate, update if new measurement, and log filtered state."""
    global robot_kf, human_kf, robot_log, human_log

    t_now = event.current_real.to_sec()

    # --- Robot ---
    if robot_kf.x is None:
        if robot_kf.has_new_measurement and robot_kf.z_time is not None:
            x, y, yaw, roll, pitch, z = robot_kf.z_latest
            robot_kf.initialize(robot_kf.z_time, x, y, yaw)
            robot_kf.has_new_measurement = False
            # immediately log the initialized state
            st = robot_kf.current_state()
            if st is not None:
                robot_log.append({
                    "t": t_now,
                    "x_f": st[0], "y_f": st[1], "theta_f": st[2],
                    "vx_f": st[3], "vy_f": st[4], "omega_f": st[5],
                    "raw_t": robot_kf.z_time, "raw_x": robot_kf.z_latest[0],
                    "raw_y": robot_kf.z_latest[1], "raw_theta": robot_kf.z_latest[2]
                })
    else:
        robot_kf.step_predict_only(t_now)
        if robot_kf.has_new_measurement:
            x, y, yaw, roll, pitch, z = robot_kf.z_latest
            robot_kf.step_update_only(x, y, yaw)
            robot_kf.has_new_measurement = False
        else:
            if robot_kf.z_latest is not None:
                roll = robot_kf.z_latest[3]; pitch = robot_kf.z_latest[4]; z = robot_kf.z_latest[5]
            else:
                roll = pitch = z = 0.0

        st = robot_kf.current_state()
        if st is not None:
            robot_log.append({
                "t": t_now,
                "x_f": st[0], "y_f": st[1], "theta_f": st[2],
                "vx_f": st[3], "vy_f": st[4], "omega_f": st[5],
                "raw_t": robot_kf.z_time if robot_kf.z_time is not None else np.nan,
                "raw_x": robot_kf.z_latest[0] if robot_kf.z_latest is not None else np.nan,
                "raw_y": robot_kf.z_latest[1] if robot_kf.z_latest is not None else np.nan,
                "raw_theta": robot_kf.z_latest[2] if robot_kf.z_latest is not None else np.nan
            })

    # --- Human ---
    if human_kf.x is None:
        if human_kf.has_new_measurement and human_kf.z_time is not None:
            x, y, yaw, roll, pitch, z = human_kf.z_latest
            human_kf.initialize(human_kf.z_time, x, y, yaw)
            human_kf.has_new_measurement = False
            st = human_kf.current_state()
            if st is not None:
                human_log.append({
                    "t": t_now,
                    "x_f": st[0], "y_f": st[1], "theta_f": st[2],
                    "vx_f": st[3], "vy_f": st[4], "omega_f": st[5],
                    "raw_t": human_kf.z_time, "raw_x": human_kf.z_latest[0],
                    "raw_y": human_kf.z_latest[1], "raw_theta": human_kf.z_latest[2]
                })
    else:
        human_kf.step_predict_only(t_now)
        if human_kf.has_new_measurement:
            x, y, yaw, roll, pitch, z = human_kf.z_latest
            human_kf.step_update_only(x, y, yaw)
            human_kf.has_new_measurement = False
        else:
            if human_kf.z_latest is not None:
                roll = human_kf.z_latest[3]; pitch = human_kf.z_latest[4]; z = human_kf.z_latest[5]
            else:
                roll = pitch = z = 0.0

        st = human_kf.current_state()
        if st is not None:
            human_log.append({
                "t": t_now,
                "x_f": st[0], "y_f": st[1], "theta_f": st[2],
                "vx_f": st[3], "vy_f": st[4], "omega_f": st[5],
                "raw_t": human_kf.z_time if human_kf.z_time is not None else np.nan,
                "raw_x": human_kf.z_latest[0] if human_kf.z_latest is not None else np.nan,
                "raw_y": human_kf.z_latest[1] if human_kf.z_latest is not None else np.nan,
                "raw_theta": human_kf.z_latest[2] if human_kf.z_latest is not None else np.nan
            })


def save_logs_to_csv():
    stamp = int(_time.time())
    base = "pose_data_filtered"
    robot_file = "{}_robot_{}.csv".format(base, stamp)
    human_file = "{}_human_{}.csv".format(base, stamp)

    def write(path, rows):
        if not rows:
            rospy.logwarn("No rows to save for %s" % path)
            with open(path, "w") as f:
                writer = csv.writer(f)
                writer.writerow(["t","x_f","y_f","theta_f","vx_f","vy_f","omega_f","raw_t","raw_x","raw_y","raw_theta"])
            return
        keys = list(rows[0].keys())
        desired = ["t","x_f","y_f","theta_f","vx_f","vy_f","omega_f","raw_t","raw_x","raw_y","raw_theta"]
        cols = [k for k in desired if k in keys] + [k for k in keys if k not in desired]
        with open(path, "w") as f:
            writer = csv.DictWriter(f, fieldnames=cols)
            writer.writeheader()
            writer.writerows(rows)

    try:
        write(robot_file, robot_log)
        write(human_file, human_log)
        rospy.loginfo("Saved logs: %s, %s" % (robot_file, human_file))
    except Exception as e:
        rospy.logerr("Failed to save logs: %s" % e)


def main():
    global robot_kf, human_kf

    rospy.init_node("pose_kf_node", anonymous=True)

    # KF tuning via params (optional)
    q_pos = rospy.get_param("~q_pos", 1e-4)
    q_theta = rospy.get_param("~q_theta", 1e-4)
    q_vel = rospy.get_param("~q_vel", 1e-2)
    q_omega = rospy.get_param("~q_omega", 1e-2)
    r_pos = rospy.get_param("~r_pos", 1e-4)
    r_theta = rospy.get_param("~r_theta", 1e-3)

    robot_kf = ConstantVelocityKF("RobotKF", q_pos, q_theta, q_vel, q_omega, r_pos, r_theta)
    human_kf = ConstantVelocityKF("HumanKF", q_pos, q_theta, q_vel, q_omega, r_pos, r_theta)

    rospy.Subscriber(ROBOT_TOPIC, PoseStamped, robot_callback)
    rospy.Subscriber(HUMAN_TOPIC, PoseStamped, human_callback)

    rate_hz = rospy.get_param("~rate_hz", 100.0)
    dt = 1.0 / float(rate_hz)
    rospy.Timer(rospy.Duration(dt), timer_event)

    rospy.on_shutdown(save_logs_to_csv)

    rospy.loginfo("pose_kf_node (logging-only) started. Subscribing to: %s , %s" % (ROBOT_TOPIC, HUMAN_TOPIC))
    rospy.loginfo("KF timer rate: %.1f Hz" % rate_hz)

    rospy.spin()


if __name__ == "__main__":
    main()
