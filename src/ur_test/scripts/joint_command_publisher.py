#!/usr/bin/env python

import rospy

from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

rospy.init_node("joint_command_publisher")

pub = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory,queue_size=10)

rate = rospy.Rate(2)

while not rospy.is_shutdown():
    jointTraj = JointTrajectory()

    point = JointTrajectoryPoint()
    jointTraj.points
    jointTraj.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    point.positions = [0, 0.38, 1.59, -0.21, 0.87, -1.30]
    
    jointTraj.points=[point]
    
    pub.publish(jointTraj)

    rate.sleep()