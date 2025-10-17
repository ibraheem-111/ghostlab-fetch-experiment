#!/usr/bin/env python
import rospy
import time
import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse
from gazebo_msgs.msg import ModelStates
# from chart_fetch.scripts.fetch_move_clients import MoveFetch


#################################################################
#################################################################
#################################################################
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import trajectory_msgs.msg


import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Move base using navigation stack

class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        result = self.client.wait_for_result()

        return result


class GripperActionClient(object):
    def __init__(self):

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

    def move_gripper(self, gripper_x, max_effort, timeout=5.0):

        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = max_effort
        gripper_goal.command.position = gripper_x

        self.gripper_client.send_goal(gripper_goal)
        result = self.gripper_client.wait_for_result(rospy.Duration(timeout))

        return result


class MoveFetch(object):

    def __init__(self):

        rospy.loginfo("In Move Fetch Calss init...")

        # Init Torso Action
        self.torso_action = FollowTrajectoryClient(
            "torso_controller", ["torso_lift_joint"])

        # Gripper Action
        self.gripper_action = GripperActionClient()

        # Point Head action
        self.head_action = PointHeadClient()

        # MOVEIT
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")

        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")

        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        self.group = moveit_commander.MoveGroupCommander("arm")
        rospy.logdebug("MoveGroupCommander for arm initialised...DONE")

        rospy.logwarn("self.group TYPE==>"+str(type(self.group)))

        rospy.loginfo("FETCH ready to move!")

    def move_manager(self, pose_requested, joints_array_requested, movement_type_requested):

        success = False

        if movement_type_requested == "TCP":
            success = self.ee_traj(pose_requested)
        elif movement_type_requested == "JOINTS":
            success = self.joint_traj(joints_array_requested)
        elif movement_type_requested == "TORSO":
            torso_height = joints_array_requested[0]
            success = self.move_torso(torso_height)
        elif movement_type_requested == "HEAD":
            XYZ = [joints_array_requested[0],
                   joints_array_requested[1],
                   joints_array_requested[2]]
            success = self.move_head_point(XYZ)
        elif movement_type_requested == "GRIPPER":
            gripper_x = joints_array_requested[0]
            max_effort = joints_array_requested[1]

            success = self.move_gripper(gripper_x, max_effort)
        else:
            rospy.logerr("Asked for non supported movement type==>" +
                         str(movement_type_requested))

        return success

    def ee_traj(self, pose):

        pose_frame = self.group.get_pose_reference_frame()

        if pose_frame != "base_link":
            new_reference_frame = "base_link"
            self.group.set_pose_reference_frame(new_reference_frame)

            pose_frame = self.group.get_pose_reference_frame()

        else:
            pass

        self.group.set_pose_target(pose)

        result = self.execute_trajectory()

        return result

    def joint_traj(self, positions_array):

        self.group_variable_values = self.group.get_current_joint_values()
        rospy.logdebug("Group Vars:")
        rospy.logdebug(self.group_variable_values)
        rospy.logdebug("Point:")
        rospy.logdebug(positions_array)
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result = self.execute_trajectory()

        return result

    def move_torso(self, torso_height):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.torso_action.move_to([torso_height, ])

        return result

    def move_gripper(self, gripper_x, max_effort):
        """
        Moves the gripper to given pose
        """
        result = self.gripper_action.move_gripper(gripper_x, max_effort)

        return result

    def move_head_point(self, XYZ, frame="base_link"):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.head_action.look_at(XYZ[0], XYZ[1], XYZ[2], frame)

        return result

    def execute_trajectory(self):

        self.plan = self.group.plan()
        result = self.group.go(wait=True)

        return result

    def ee_pose(self):

        gripper_pose = self.group.get_current_pose()

        rospy.logdebug("EE POSE==>"+str(gripper_pose))

        return gripper_pose

    def ee_rpy(self, request):

        gripper_rpy = self.group.get_current_rpy()

        return gripper_rpy



#################################################################
#################################################################
#################################################################
class BumbleBlock_Subscriber(object):
    def __init__(self):
        self.msg_type = Pose
        self.msg_name = "/vrpn_client_node/BumbleBall/pose"

        self.position = [0,0,0] # x,y,z cartesian
        self.orientation = [0,0,0,1] # rx,ry,rz,rw quat

        # rospy.init_node('BB_listener', anonymous=True)
        rospy.Subscriber(self.msg_name, self.msg_type, self.callback)
    
    def is_ready(self):
        return np.any(np.array(self.position)!=0)
    


    def callback(self,msg):
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.orientation = [msg.orientation.x,msg.orientation.y,  msg.orientation.z,msg.orientation.w]
        # rospy.loginfo(self.position)
   
#################################################################
#################################################################
#################################################################
class FetchVelocityController(object):
    """
    whenever deadman is held cmd_vel/teleop will override cmd_vel.
    """
    def __init__(self):
        # Subscribe to current position (mocap/odom)
        # only if child_frame_id = "base_link"
        # rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_localization)

        rospy.Subscriber('/vrpn_client_node/FetchBase/pose', Pose, self.callback_localization)

        self.curr_pose = Pose()
        # self.curr_twist = Twist()
        self.position = [0.0, 0.0, 0.0] # x,y,z cartesian
        self.orientation = [0, 0, 0, 1] # rx,ry,rz,rw quat
        self.curr_yaw = 0.0 # rx,ry,rz euler

        # Create publisher to fetch base velocity controller
        self.pub_base_vel = rospy.Publisher("cmd_vel",Twist, queue_size=10)

        # self.move_fetch_obj =  MoveFetch()

        
    # PUBLISHERS #############################################
    def cmd_base_twist(self,twist):
        self.pub_base_vel.publish(twist)

    def cmd_base_forward(self,speed):
        twist = Twist()
        twist.linear.x = speed
        self.pub_base_vel.publish(twist)

    def cmd_base_spin(self,speed):
        twist = Twist()
        twist.angular.z = speed
        self.pub_base_vel.publish(twist)

    def cmd_base_stop(self):
        self.pub_base_vel.publish(Twist())

    def cmd_base_approach(self,goal_position):
        # Setting
        lin_speed = 0.4
        spin_speed = 0.45
        tol_yaw_straight = 3
        tol_yaw_curve = 8

        # Calc diff in position and approach angle
        dx,dy,_ = np.array(goal_position)-np.array(self.position)
        ddist = np.linalg.norm([dx,dy])
        yaw_offset = -180 if np.sign(dx) ==-1 and np.sign(dy) == -1 else 0
        goal_yaw = np.arctan(dy/dx)*(180.0/np.pi) + yaw_offset
        curr_yaw = self.curr_yaw*(180.0/np.pi)
        dyaw = goal_yaw - curr_yaw
        if dyaw >= 360.0: dyaw -= 360.0
        if dyaw <= -360.0: dyaw += 360.0

        # Approach
        rospy.loginfo([[goal_yaw,curr_yaw, dyaw]])
        rospy.loginfo([dx, dy, ddist])

        at_goal = False
        if np.isnan([dyaw, ddist]).any():
            rospy.loginfo("^^ bad value ^^")
            self.cmd_base_stop()
        elif abs(ddist) < tol_dist:
            rospy.loginfo(">> at goal")
            at_goal=True        
        elif abs(dyaw) < tol_yaw_straight:
            rospy.loginfo(">>fwd")
            self.cmd_base_forward(lin_speed)
        elif abs(dyaw) < tol_yaw_curve:
            rospy.loginfo(">>curve")
            twist = Twist()
            twist.linear.x = lin_speed
            twist.angular.z = spin_speed*np.sign(dyaw)/2
            self.cmd_base_twist(twist)
        else:
            rospy.loginfo(">>spin")
            speed = spin_speed*np.sign(dyaw)
            self.cmd_base_spin(speed)

        return at_goal



    # def cmd_tcp_pose(self,tcp_pose):
    #     self.move_fetch_obj.move_manager(pose_requested=tcp_pose,
    #                                 joints_array_requested=[],
    #                                 movement_type_requested="TCP")
    # def cmd_gripper(self,status):
    #     max_effort = 10.0
    #     if status == "open":  grip_position = 0.5
    #     elif status == "close": gripper_position = 0.02
    #     else: rospy.logerr('INVALID cmd_gripper in Fetch Velocity Controller')

    #     self.move_fetch_obj.move_manager(pose_requested=None,
    #                                     joints_array_requested=[grip_position, max_effort],
    #                                     movement_type_requested="GRIPPER")
            
    


    # CALLBACKS ##############################################
    def callback_localization(self,msg):
        # For VRPN localization ------------------------------
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.orientation = [msg.orientation.x,msg.orientation.y,  msg.orientation.z,msg.orientation.w]
        self.curr_yaw = euler_from_quaternion(Kfetch.orientation)[-1]

        # For GAZEBO localization ------------------------------
        # if 'fetch' in msg.name:
        #     self.curr_pose = msg.pose[-1]
        #     self.curr_twist = msg.twist[-1]
        #     self.position = [self.curr_pose.position.x,
        #                      self.curr_pose.position.y,
        #                      self.curr_pose.position.z]
        #     self.orientation = [self.curr_pose.orientation.x,
        #                         self.curr_pose.orientation.y,
        #                         self.curr_pose.orientation.z,
        #                         self.curr_pose.orientation.w]
        #     self.curr_yaw = euler_from_quaternion(Kfetch.orientation)[-1]

        # # For ODOM localization ------------------------------
        # if msg.child_frame_id == "base_link":
        #     self.curr_pose = msg.pose.pose
        #     self.curr_twist = msg.twist.twist
        #     self.position = [msg.pose.pose.position.x,
        #                      msg.pose.pose.position.y,
        #                      msg.pose.pose.position.z]
        #     self.orientation = [msg.pose.pose.orientation.x,
        #                         msg.pose.pose.orientation.y,
        #                         msg.pose.pose.orientation.z,
        #                         msg.pose.pose.orientation.w]
        #     self.curr_yaw = euler_from_quaternion(Kfetch.orientation)[-1]


def Rz(theta):
  return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])
#################################################################
#################################################################
#################################################################
if __name__=="__main__":
    tol_yaw = 1.0
    tol_dist = 0.7

    # init node
    rospy.init_node("fetch_vel_controller")
    rate = rospy.Rate(50) # 10 Hz

    # Initialize controller
    Kfetch = FetchVelocityController()
    # Setup clients
    move_fetch_obj = MoveFetch()
    BB = BumbleBlock_Subscriber()


    # Setup Params
    ready2navigate = True
    ready2pick = False
    ready2deliver = False
    ready2place = False

    lin_speed = 0.4
    spin_speed = 0.45

    quat_point_down = quaternion_from_euler(ai=0.0, aj=1.57, ak=0.0)  
    grip_max_effort = 10.0  
    
    z_above = 0.65
    z_pick_offset = 0.25

    x_tuck = 0.4
    y_tuck = 0.0

    x_far = 0.6
    y_far = 0.0

    x_offset = -0.1
    rospy.sleep(2)
    # rospy.loginfo('Waiting for BumbleBall...')
    # while not rospy.is_shutdown():
    #     if BB.is_ready:
    #         break
    #     rate.sleep() # sleep node to avoid freezes
    # rospy.loginfo('BumbleBall ready')
    
    
    # --------------------------------------------
    # Initialte controll loop --------------------
    tstart = time.time()
    while not rospy.is_shutdown():
        # get current duration (seconds) of loop
        t = time.time() - tstart 

       

        ###########################################
        # Navigation Logic ########################
        if ready2navigate:
            at_goal = Kfetch.cmd_base_approach(BB.position)
            if at_goal:
                Kfetch.cmd_base_stop()
                ready2navigate = False
                ready2pick = True
                ready2deliver = False
                ready2place = False

        if ready2pick:

            # Calculate relative transforms between Fetch and BB for MoveIt! commands
            dpos =  np.array(BB.position) -np.array(Kfetch.position)
            dquat = np.array(Kfetch.orientation) #* np.array(quaternion_inverse(BB.orientation))
            deuler = np.array(euler_from_quaternion(dquat))
            dyaw = deuler[-1]
            dpos = dpos*Rz(dyaw) # rotate pos vector by fetch rotation
            dpos = dpos.tolist()[0] # flatten to 1d list

            rospy.loginfo('\n\n BB relative position...')
            rospy.loginfo(dpos)

            # Set TCP Pose to BB location
            tcp_pose = Pose()
            tcp_pose.position.x = dpos[0] + x_offset
            tcp_pose.position.y = dpos[1]
            tcp_pose.position.z = dpos[2]    
            tcp_pose.orientation.x = quat_point_down[0]
            tcp_pose.orientation.y = quat_point_down[1]
            tcp_pose.orientation.z = quat_point_down[2]
            tcp_pose.orientation.w = quat_point_down[3]


            # Move TCP to above BB
            tcp_pose.position.z = z_above
            move_fetch_obj.move_manager(pose_requested=tcp_pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")
            

            # Open Gripper
            grip_position = 0.5
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=[grip_position, grip_max_effort],
                                        movement_type_requested="GRIPPER")


            # Move TCP to BB 
            tcp_pose.position.z = z_pick_offset
            move_fetch_obj.move_manager(pose_requested=tcp_pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")
            
            # Close Gripper
            grip_position = 0.02
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=[grip_position, grip_max_effort],
                                        movement_type_requested="GRIPPER")
            
            # Move TCP to above BB
            tcp_pose.position.z = z_above
            move_fetch_obj.move_manager(pose_requested=tcp_pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")
            

            # Tuck Arm in for transport
            tcp_pose.position.x = x_tuck
            tcp_pose.position.y = y_tuck
            move_fetch_obj.move_manager(pose_requested=tcp_pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")
            
            # Change state machine condition
            ready2navigate = False                                
            ready2pick = False
            ready2deliver = True
            ready2place = False

        ###########################################
        # Deliver BumbleBlock #####################
        if ready2deliver:
            rospy.loginfo("\n\n####### begining delivery ######")
            place_position = [0.0,0.0,0.0]

            at_goal = Kfetch.cmd_base_approach(place_position)
            if at_goal:
                Kfetch.cmd_base_stop()
                rospy.loginfo("Delivered...\n\n")
                ready2navigate = False
                ready2pick = False
                ready2deliver = False
                ready2place = True

             # Calculate relative metrics
            # place_position = [0.0,0.0,0.0]
            # dx,dy,_ = np.array(place_position)-np.array(Kfetch.position)
            # goal_yaw = np.arctan(dy/dx)*(180.0/np.pi) - 180 
            # curr_yaw = Kfetch.curr_yaw*(180.0/np.pi)
            # dyaw = goal_yaw - curr_yaw
            # ddist = np.linalg.norm([dx,dy])

            # rospy.loginfo([[dx,dy,ddist],[goal_yaw,curr_yaw, dyaw]])
            # rospy.loginfo([[dx,dy,ddist]])
            # rospy.loginfo([[goal_yaw,curr_yaw, dyaw]])

            
            # # if dyaw > np.pi: dyaw -= np.pi
            # if dyaw >= 360.0: dyaw -= 360.0
            # if dyaw <= -360.0: dyaw += 360.0

            # if np.isnan([dyaw, ddist]).any():
            #     rospy.loginfo("^^ bad value ^^")
            #     Kfetch.cmd_base_stop()
            # elif abs(dyaw) > tol_yaw :
            #     rospy.loginfo(">>spin")
            #     # rospy.loginfo([goal_yaw, curr_yaw, dyaw])
            #     speed = spin_speed*np.sign(dyaw)
            #     Kfetch.cmd_base_spin(speed)
            # elif abs(ddist) > tol_dist and abs(dyaw) > tol_yaw*0.5:
            #     twist = Twist()
            #     twist.linear.x = lin_speed
            #     twist.angular.z = spin_speed*np.sign(dyaw)/2
            #     Kfetch.cmd_base_twist(twist)
            # elif abs(ddist) > tol_dist:
            #     rospy.loginfo(">>fwd")
            #     # rospy.loginfo([dx, dy, ddist])
            #     Kfetch.cmd_base_forward(lin_speed)
            # else: 
            #     Kfetch.cmd_base_stop()
            #     rospy.loginfo("Delivered...\n\n")
            #     ready2navigate = False
            #     ready2pick = False
            #     ready2deliver = False
            #     ready2place = True
        # ###########################################
        # # Place BumbleBlock #####################

        if ready2place:
            place_position = [0,0,0]

            # Calculate relative transforms between Fetch and BB for MoveIt! commands
            dpos =  np.array(place_position) -np.array(Kfetch.position)
            dquat = np.array(Kfetch.orientation) #* np.array(quaternion_inverse(BB.orientation))
            deuler = np.array(euler_from_quaternion(dquat))
            dyaw = deuler[-1]
            dpos = dpos*Rz(dyaw) # rotate pos vector by fetch rotation
            dpos = dpos.tolist()[0] # flatten to 1d list


            # Set TCP Pose to BB location
            tcp_pose = Pose()
            tcp_pose.position.x = dpos[0] + x_offset
            tcp_pose.position.y = dpos[1]
            tcp_pose.position.z = dpos[2]    
            tcp_pose.orientation.x = quat_point_down[0]
            tcp_pose.orientation.y = quat_point_down[1]
            tcp_pose.orientation.z = quat_point_down[2]
            tcp_pose.orientation.w = quat_point_down[3]


            # Move TCP to above place_position
            tcp_pose.position.z = z_above
            move_fetch_obj.move_manager(pose_requested=tcp_pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")
        


            # Move TCP to place_position 
            tcp_pose.position.z = z_pick_offset
            move_fetch_obj.move_manager(pose_requested=tcp_pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")
            
            # Open Gripper
            grip_position = 0.5
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=[grip_position, grip_max_effort],
                                        movement_type_requested="GRIPPER")
            
            # Move TCP to above place_position
            tcp_pose.position.z = z_above
            move_fetch_obj.move_manager(pose_requested=tcp_pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")
            

            # Tuck Arm in for transport
            tcp_pose.position.x = x_tuck
            tcp_pose.position.y = y_tuck
            move_fetch_obj.move_manager(pose_requested=tcp_pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")
            
            # Change State Machine Conditions

            ready2navigate = False                               
            ready2pick = False
            ready2deliver = False
            ready2place = False

            # Break when done
            break

        rate.sleep() # sleep node to avoid freezes
    rospy.spin() # idle ros node