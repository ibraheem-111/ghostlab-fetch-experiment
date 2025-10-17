#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

# python imports
import math
import time

#######################################################
# RIDGED BODY OBJECT ##################################
#######################################################
class ridged_body_simulator(object):
    def __init__(self,node_name,ridged_body_name):
        self.message_name = 'pose'
        self.node_name = node_name
        self.ridged_body_name = ridged_body_name
        self.full_msg_name = self.node_name + '/' + self.ridged_body_name + '/' + self.message_name

        # Settings
        self.msg_type = Pose#String
        self.pub = rospy.Publisher(self.full_msg_name,self.msg_type , queue_size=10)
        self.pose = Pose() # return ros pose pessage

    def update_pose(self,pose_lst):
        """ param: pose_lst - list of pose [x,y,z,rx,ry,rz,rw]  """

        # Format position
        self.pose.position.x = pose_lst[0]
        self.pose.position.y = pose_lst[1]
        self.pose.position.z = pose_lst[2]

        # Format orientation
        self.pose.orientation.x = pose_lst[3]
        self.pose.orientation.y = pose_lst[4]
        self.pose.orientation.z = pose_lst[5]
        self.pose.orientation.w = pose_lst[6]


    def publish(self,verbose=False):
        self.pub.publish(self.pose) # publish msg to topic
        if verbose: rospy.loginfo(self.pose) # report in consol



#######################################################
# RUN MAIN LOOP PROCESSES #############################
#######################################################
def main():
    # Define node-level parameters
    node_name =  "vrpn_client_node"
    rospy.init_node(node_name, anonymous=True)
    rate = rospy.Rate(100) # 10 Hz

    # Initialize ridged body simulator
    sim_BB = ridged_body_simulator(node_name,'BumbleBall')

    # Run simulation loop
    tstart = time.time()
    while not rospy.is_shutdown():
        # get current duration (seconds) of loop
        t = time.time() - tstart 

        # Calculate simulated pos for BumbleBall
        pos = [3.0,3.0,0] # pos = [math.sin(t),math.cos(t),0]
        # pos = [0.0,0.0,0] # pos = [math.sin(t),math.cos(t),0]
        quat = [0,0,0,1.0] # quaternion
        pose = pos + quat
        sim_BB.update_pose(pose)
        sim_BB.publish()

        rate.sleep() # sleep node to avoid freezes



if __name__ == '__main__':
    main()
