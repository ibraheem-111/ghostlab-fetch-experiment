#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def publish_cmd_vel():
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)  # 3 Hz

    msg = Twist()
    msg.linear.x = 0.2
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    rospy.loginfo("Publishing /cmd_vel messages at 50Hz...")
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_cmd_vel()
    except rospy.ROSInterruptException:
        pass

