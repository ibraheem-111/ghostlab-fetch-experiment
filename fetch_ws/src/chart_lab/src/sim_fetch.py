#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import time
from gazebo_msgs.msg import ModelStates
# Take gazebo data and stream to VRPN topic

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

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_localization)


    def callback_localization(self,msg):
        # For GAZEBO localization
        if 'fetch' in msg.name:
            self.pose =  msg.pose[-1]

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
    sim_fetch = ridged_body_simulator(node_name,'FetchBase')

    # Run simulation loop
    tstart = time.time()
    while not rospy.is_shutdown():
        # get current duration (seconds) of loop
        t = time.time() - tstart 
        sim_fetch.publish()
        rate.sleep() # sleep node to avoid freezes

if __name__ == '__main__':
    main()
