#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os
import argparse

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS messages types of the real robot
from iiwa_msgs.msg import JointPosition, JointQuantity

# ------------------------------------------------------------
#  REAL ROBOT
# ------------------------------------------------------------
FREQ = 100
NDOF = 7
REAL_ROBOT_TARGET_JOINT_COMMAND_TOPIC = 'command/JointPosition' # commands joint states on this topic
REAL_ROBOT_JOINT_STATE_TOPIC = 'state/JointPosition'

min_joints = [-169, -100, -169, -119, -169, -119, -173]
max_joints = [ 169,  100,  169,  119,  169,  119, 173]

# number of interpolation steps
NUM_STEPS  = 100


def go2initial(robot_name=None, target_joint_position = np.asarray([0,0,0,0,0,0,0])):

    if robot_name==None:
        rospy.logerr(f"No robot name was given.")
        return False

    # Setup ros name for topics
    real_world_publishers_topic_name = f"{robot_name}/{REAL_ROBOT_TARGET_JOINT_COMMAND_TOPIC}"
    real_world_target_joint_command_publisher = rospy.Publisher(real_world_publishers_topic_name, JointPosition, queue_size=1)

    # init node
    rospy.init_node('go2initial_configuration')
    r = rospy.Rate(FREQ) # 10hz

    # read the current configuration of the robot
    rospy.loginfo(f"Waiting for {robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC} topic, to read the curent configuration of the robot.")
    msgRobotState = rospy.wait_for_message(f"{robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC}", JointPosition)
    cur_joint_pos = msgRobotState.position
    current_joint_position = np.asarray([cur_joint_pos.a1,
                                         cur_joint_pos.a2,
                                         cur_joint_pos.a3,
                                         cur_joint_pos.a4,
                                         cur_joint_pos.a5,
                                         cur_joint_pos.a6,
                                         cur_joint_pos.a7])

    # transform degrees configuration to rads
    target_joint_position_rad = np.deg2rad(target_joint_position)
    # # interpolate between current configuration and the goal configuration
    joint_values = np.linspace(current_joint_position, target_joint_position_rad, NUM_STEPS)

    # initial the message
    msg = JointPosition()
    msg.header.frame_id = "Joint_position"

    joint_pos_submsg = JointQuantity()

    # move robot to target config by publishing robot configuration states
    for i in range(4,NUM_STEPS):

        msg.header.stamp = rospy.Time.now()

        # add data as flattened numpy array
        cmd_joints = joint_values[i,:]

        joint_pos_submsg.a1 = cmd_joints[0]
        joint_pos_submsg.a2 = cmd_joints[1]
        joint_pos_submsg.a3 = cmd_joints[2]
        joint_pos_submsg.a4 = cmd_joints[3]
        joint_pos_submsg.a5 = cmd_joints[4]
        joint_pos_submsg.a6 = cmd_joints[5]
        joint_pos_submsg.a7 = cmd_joints[6]

        msg.position = joint_pos_submsg

        # send to the robot
        real_world_target_joint_command_publisher.publish(msg)
        r.sleep()


if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Description of your program')
    parser.add_argument("--robot_name", help="Give the name of the robot")
    parser.add_argument("--target_config", nargs="+", help="Give the target configuration of the robot", type=float)
    args = parser.parse_args()

    if args.target_config:
        goal_q = np.asarray(list(args.target_config))
        rows_target = goal_q.shape[0]
        if rows_target != NDOF:
            rospy.logerr(f"Target configuration was given to the robot has wrong dimensions")
        else:

            np.clip(goal_q, min_joints, max_joints, out=goal_q)
            go2initial(args.robot_name, goal_q)

    else:
        rospy.loginfo(f"No target configuration was given to the robot.")


 # end
