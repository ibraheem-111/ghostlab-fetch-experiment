#!/usr/bin/env python
"""
pose_collector.py

Subscribe to two PoseStamped topics (robot & human), convert orientation
quaternion -> euler, store data in memory, and write two CSVs on shutdown:
one for the robot, one for the human.

Used for HRI experiments where both are modeled as rigid bodies.
"""

import rospy
import csv
import tf
from geometry_msgs.msg import PoseStamped
import time as _time  # keep the module separate from 'time' field names


# Store samples separately for each rigid body
robot_data = []
human_data = []


FIELDS = ["x", "y", "z", "alpha", "beta", "gamma", "time"]


def pose_msg_to_dict(msg):
    """
    Convert a PoseStamped message into a dict with the desired keys.
    """
    stamp_secs = msg.header.stamp.to_sec()
    pose = msg.pose

    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )

    try:
        alpha, beta, gamma = tf.transformations.euler_from_quaternion(quaternion)
    except Exception as e:
        rospy.logwarn("Failed to transform quaternion to euler: %s", e)
        return None

    data = {
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z,
        "alpha": alpha,
        "beta": beta,
        "gamma": gamma,
        "time": stamp_secs
    }

    return data


def robot_callback(msg):
    data = pose_msg_to_dict(msg)
    if data is None:
        return

    robot_data.append(data)
    rospy.loginfo_throttle(2.0, "Robot: collected %d samples", len(robot_data))


def human_callback(msg):
    data = pose_msg_to_dict(msg)
    if data is None:
        return

    human_data.append(data)
    rospy.loginfo_throttle(2.0, "Human: collected %d samples", len(human_data))


def write_csv(filename, data_list, label):
    if not data_list:
        rospy.logwarn("%s: no data collected, writing header-only CSV (%s)", label, filename)

    try:
        with open(filename, "w") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(data_list)
        rospy.loginfo("%s: saved %d entries to %s", label, len(data_list), filename)
    except Exception as e:
        rospy.logerr("%s: failed to save CSV: %s", label, e)


def save_to_csv():
    """
    Called automatically on node shutdown.
    Writes one CSV for robot and one for human.
    """
    rospy.loginfo("Saving robot and human data to CSV...")

    # Use a shared timestamp so the pair of files is clearly from the same run
    current_time = int(_time.time())
    base_name = "pose_data_movement_50Hz"

    robot_file = "{}_robot_{}.csv".format(base_name, current_time)
    human_file = "{}_human_{}.csv".format(base_name, current_time)

    write_csv(robot_file, robot_data, "Robot")
    write_csv(human_file, human_data, "Human")

    rospy.loginfo("Done saving CSVs. Exiting gracefully.")


def main():
    rospy.init_node('pose_collector', anonymous=True)

    # Private parameters (can be overridden via roslaunch/rosrun)
    robot_topic = rospy.get_param('~robot_topic', '/vrpn_client_node/Fetch/pose')
    human_topic = rospy.get_param('~human_topic', '/vrpn_client_node/Human/pose')

    rospy.loginfo("Subscribing to robot topic: %s", robot_topic)
    rospy.loginfo("Subscribing to human topic: %s", human_topic)

    rospy.Subscriber(robot_topic, PoseStamped, robot_callback)
    rospy.Subscriber(human_topic, PoseStamped, human_callback)

    rospy.on_shutdown(save_to_csv)

    rospy.loginfo("Listening... Press Ctrl+C to stop the experiment.")
    rospy.spin()


if __name__ == '__main__':
    main()
