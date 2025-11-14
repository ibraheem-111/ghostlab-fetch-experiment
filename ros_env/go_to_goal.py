#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, Twist

class GoToGoal:
    def __init__(self):
        rospy.init_node('go_to_goal_controllerp', anonymous=True)

        # Parameters
        self.target_x = rospy.get_param("~target_x", -1.0)   # meters
        self.target_y = rospy.get_param("~target_y", 1.0)
        self.linear_k = rospy.get_param("~linear_k", 0.5)
        self.angular_k = rospy.get_param("~angular_k", 1.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.05)

        self.pose_sub = rospy.Subscriber("/vrpn_client_node/RigidBody004/pose", PoseStamped, self.pose_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.current_pose = None
        self.rate = rospy.Rate(50)  # 50 Hz control loop
        rospy.loginfo("GoToGoal node started. Target = (%.2f, %.2f)", self.target_x, self.target_y)

    def pose_callback(self, msg):
        pose = msg.pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.current_pose = (pose.position.x, pose.position.y, yaw)

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.current_pose is None:
                self.rate.sleep()
                continue

            x, y, yaw = self.current_pose

            # Compute control errors
            dx = self.target_x - x
            dy = self.target_y - y
            distance = math.sqrt(dx**2 + dy**2)
            target_heading = math.atan2(dy, dx)
            heading_error = target_heading - yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # normalize

            cmd = Twist()

            if distance > self.goal_tolerance:
                cmd.linear.x = self.linear_k * distance
                cmd.angular.z = self.angular_k * heading_error
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                rospy.loginfo("Reached goal (%.2f, %.2f)", self.target_x, self.target_y)
                self.cmd_pub.publish(cmd)
                break

            # Limit speeds for safety
            cmd.linear.x = max(min(cmd.linear.x, 0.4), -0.4)
            cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = GoToGoal()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
