import rospy
import csv
import tf
from geometry_msgs.msg import PoseStamped
import time

data_list = []

def callback(msg):
    stamp = msg.header.stamp
    pose = msg.pose

    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    alpha, beta, gamma = euler
    
    time = stamp.secs

    data = {
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z,
        "alpha": alpha,
        "beta": beta,
        "gamma": gamma,
        "time": time
    }

    data_list.append(data)
    rospy.loginfo("Collected: %s", data)

def save_to_csv():
    rospy.loginfo("Saving data to CSV...")
    current_time = time.time()
    file_name = "pose_data_movement_50Hz" + str(current_time) + ".csv"
    with open(file_name, "w") as f:
        writer = csv.DictWriter(f, fieldnames=["x", "y", "z", "alpha", "beta", "gamma", "time"])
        writer.writeheader()
        writer.writerows(data_list)
    rospy.loginfo("Saved {} entries to {}".format(str(len(data_list)), file_name))
    rospy.loginfo("Exiting gracefully.")

def listener(ros_topic):
    rospy.init_node('pose_collector', anonymous=True)
    rospy.Subscriber(ros_topic, PoseStamped, callback)
    rospy.on_shutdown(save_to_csv)
    rospy.loginfo("Listening to {} ... Press Ctrl+C to stop.".format(ros_topic))
    rospy.spin()

if __name__ == '__main__':
    topic = "/vrpn_client_node/fetch/pose"
    listener(topic)
