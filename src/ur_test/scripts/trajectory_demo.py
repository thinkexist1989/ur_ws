#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')

        reset = rospy.get_param('~reset',False)

        arm_joints=['joint_1',
                    'joint_2',
                    'joint_3',
                    'joint_4',
                    'joint_5',
                    'joint_6']

        if reset:
            arm_goal = [0, 0, 0, 0, 0, 0]
        else:
            arm_goal = [0.09, 0.26, 1.02, 0, -1.14, 3.24]

        
        rospy.loginfo('Waiting for arm trajectory controller...')
        arm_client = actionlib.SimpleActionClient('/joint_trajectory_action',FollowJointTrajectoryAction)
        arm_client.wait_for_server()
        rospy.loginfo('...connected.')

        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)

        rospy.loginfo('Moving the arm to goal position...')

        goal=FollowJointTrajectoryGoal()
        goal.trajectory=arm_trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        arm_client.send_goal(goal)

        arm_client.wait_for_result(rospy.Duration(5.0))

        rospy.loginfo('...done')
        

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass