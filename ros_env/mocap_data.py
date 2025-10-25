import rospy
import csv
import tf
from geometry_msgs.msg import PoseStamped

data_list = []

def callback(msg):
    pose = msg.pose

    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    alpha, beta, gamma = euler

    data = {
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z,
        "alpha": alpha,
        "beta": beta,
        "gamma": gamma
    }

    data_list.append(data)
    rospy.loginfo("Collected: %s", data)

def save_to_csv():
    rospy.loginfo("Saving data to CSV...")
    with open("pose_data.csv", "w") as f:
        writer = csv.DictWriter(f, fieldnames=["x", "y", "z", "alpha", "beta", "gamma"])
        writer.writeheader()
        writer.writerows(data_list)
    rospy.loginfo("Saved %d entries to pose_data.csv", len(data_list))
    rospy.loginfo("Exiting gracefully.")

def listener(ros_topic):
    rospy.init_node('pose_collector', anonymous=True)
    rospy.Subscriber(ros_topic, PoseStamped, callback)
    rospy.on_shutdown(save_to_csv)
    rospy.loginfo("Listening to {} ... Press Ctrl+C to stop.".format(ros_topic))
    rospy.spin()

if __name__ == '__main__':
    topic = "/vrpn_client_node/FetchMobile/pose"
    listener()
