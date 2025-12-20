#!/usr/bin/env python
"""
pose_collector.py

ROS1 node (Melodic) to record robot & human PoseStamped data into separate CSVs,
and to create a combined 6-element state vector on each incoming message:
[X_Human, Y_Human, Theta_Human, X_robot, Y_robot, Theta_robot, time]

Theta is the yaw (rotation about Z) in radians (tf.euler_from_quaternion -> roll,pitch,yaw).
"""

import rospy
import csv
import tf
from geometry_msgs.msg import PoseStamped
import time as _time
from threading import Lock

# Topics
ROBOT_TOPIC = "/vrpn_client_node/Fetch/pose"
HUMAN_TOPIC = "/vrpn_client_node/Human/pose"

# Buffers
robot_data = []
human_data = []
paired_states = []  # combined state vectors

# Latest known poses (dicts), set to None until populated
_latest_robot = None
_latest_human = None

# Lock for updating latest poses safely across callbacks
_lock = Lock()

# CSV fields
FIELDS = ["x", "y", "z", "alpha", "beta", "gamma", "time"]
STATE_FIELDS = ["X_Human", "Y_Human", "Theta_Human",
                "X_Robot", "Y_Robot", "Theta_Robot", "time"]


def quaternion_to_euler(pose):
    """Return roll, pitch, yaw for a geometry_msgs/Pose."""
    q = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    try:
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
    except Exception as e:
        rospy.logwarn("Quaternion->Euler failed: %s", e)
        return None, None, None
    return roll, pitch, yaw


def pose_msg_to_dict(msg):
    """Convert PoseStamped message into dict used for per-body CSVs."""
    stamp = msg.header.stamp.to_sec()
    pose = msg.pose

    roll, pitch, yaw = quaternion_to_euler(pose)
    if roll is None:
        return None

    data = {
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z,
        "alpha": roll,
        "beta": pitch,
        "gamma": yaw,
        "time": stamp
    }
    return data


def build_state_vector(robot_dict, human_dict, time_stamp):
    """
    Build [X_Human, Y_Human, Theta_Human, X_robot, Y_robot, Theta_robot, time]
    robot_dict & human_dict have keys x,y,gamma (gamma == yaw)
    """
    return {
        "X_Human": human_dict["x"],
        "Y_Human": human_dict["y"],
        "Theta_Human": human_dict["gamma"],
        "X_Robot": robot_dict["x"],
        "Y_Robot": robot_dict["y"],
        "Theta_Robot": robot_dict["gamma"],
        "time": time_stamp
    }


def robot_callback(msg):
    global _latest_robot, _latest_human
    data = pose_msg_to_dict(msg)
    if data is None:
        return

    # append per-body record
    robot_data.append(data)
    rospy.loginfo_throttle(2.0, "Robot: collected %d samples", len(robot_data))

    with _lock:
        _latest_robot = data
        # if human already known, append paired state using this message's time
        if _latest_human is not None:
            state = build_state_vector(_latest_robot, _latest_human, data["time"])
            paired_states.append(state)
            rospy.loginfo_throttle(2.0, "Paired states: %d", len(paired_states))


def human_callback(msg):
    global _latest_robot, _latest_human
    data = pose_msg_to_dict(msg)
    if data is None:
        return

    human_data.append(data)
    rospy.loginfo_throttle(2.0, "Human: collected %d samples", len(human_data))

    with _lock:
        _latest_human = data
        if _latest_robot is not None:
            # use this human message's time as timestamp for paired state
            state = build_state_vector(_latest_robot, _latest_human, data["time"])
            paired_states.append(state)
            rospy.loginfo_throttle(2.0, "Paired states: %d", len(paired_states))


def write_csv(filename, data_list, fieldnames, label):
    if not data_list:
        rospy.logwarn("%s: no data collected, writing header-only CSV (%s)", label, filename)

    try:
        with open(filename, "w") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data_list)
        rospy.loginfo("%s: saved %d entries to %s", label, len(data_list), filename)
    except Exception as e:
        rospy.logerr("%s: failed to save CSV: %s", label, e)


def save_to_csv():
    rospy.loginfo("Saving robot, human, and paired state CSVs...")

    current_time = int(_time.time())
    base = "pose_data_movement_50Hz"

    robot_file = "{}_robot_{}.csv".format(base, current_time)
    human_file = "{}_human_{}.csv".format(base, current_time)
    paired_file = "{}_statepair_{}.csv".format(base, current_time)

    write_csv(robot_file, robot_data, FIELDS, "Robot")
    write_csv(human_file, human_data, FIELDS, "Human")
    write_csv(paired_file, paired_states, STATE_FIELDS, "PairedState")

    rospy.loginfo("Done saving CSVs.")


def main():
    rospy.init_node("pose_collector", anonymous=True)

    rospy.Subscriber(ROBOT_TOPIC, PoseStamped, robot_callback)
    rospy.Subscriber(HUMAN_TOPIC, PoseStamped, human_callback)

    rospy.on_shutdown(save_to_csv)

    rospy.loginfo("Listening to:\n  Robot topic: %s\n  Human topic: %s\nPress Ctrl+C to stop.",
                  ROBOT_TOPIC, HUMAN_TOPIC)

    rospy.spin()


if __name__ == "__main__":
    main()
