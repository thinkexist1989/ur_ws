#!/usr/bin/env python

import rospy, sys
import numpy as np
import moveit_commander
from std_msgs.msg import Float64
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_ik_demo')

        arm = moveit_commander.MoveGroupCommander('manipulator')

        end_effector_link = arm.get_end_effector_link()
        rospy.loginfo("end_effector_link is: " + end_effector_link)

        reference_frame = 'base_link'

        arm.set_pose_reference_frame(reference_frame)

        arm.allow_replanning(True)

        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        arm.set_named_target('up')
        arm.go()

        up_pose = arm.get_current_pose(end_effector_link).pose
        angles = euler_from_quaternion([up_pose.orientation.x,up_pose.orientation.y,up_pose.orientation.z,up_pose.orientation.w])
        rospy.loginfo(" roll: " + str(angles[0]) + " pitch: " + str(angles[1]) + " yaw: " + str(angles[2]) )
        rospy.sleep(2)

        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = -0.6
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.5

        q = quaternion_from_euler(np.deg2rad(0), np.deg2rad(0), np.deg2rad(180))
        # q = quaternion_from_euler(0,2,-2)
        target_pose.pose.orientation = Quaternion(*q)
        # target_pose.pose.orientation.x = q[0]
        # target_pose.pose.orientation.y = q[1]
        # target_pose.pose.orientation.z = q[2]
        # target_pose.pose.orientation.w = q[3]

        arm.set_start_state_to_current_state()

        arm.set_pose_target(target_pose, end_effector_link)

        traj = arm.plan()

        arm.execute(traj)
        rospy.sleep(1)

        arm.shift_pose_target(1, -0.05, end_effector_link)
        arm.go()
        rospy.sleep(1)

        arm.shift_pose_target(3,-1.57, end_effector_link)
        arm.go()
        rospy.sleep(1)

        arm.set_named_target('up')
        arm.go()

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()