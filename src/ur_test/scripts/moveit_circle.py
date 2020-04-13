#!/usr/bin/env python

import rospy, sys, math
import numpy as np
import moveit_commander

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import RobotTrajectory
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItCartesianDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        arm = moveit_commander.MoveGroupCommander('manipulator')

        arm.allow_replanning(True)
        arm.set_max_acceleration_scaling_factor(1)
        arm.set_max_velocity_scaling_factor(1)

        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)

        end_effector_link = arm.get_end_effector_link()

        arm.set_named_target('up')
        arm.go()
        rospy.sleep(0.5)

        rospy.loginfo("Now, Moving to first point...")

        # center of the circle
        center_pose = Pose()
        center_pose.position.x = 0.7
        center_pose.position.y = 0.0
        center_pose.position.z = 0.7
        q = quaternion_from_euler(np.deg2rad(0), np.deg2rad(0), np.deg2rad(0))
        center_pose.orientation = Quaternion(*q)

        #move to first point
        first_pos = PoseStamped()
        first_pos.header.frame_id = reference_frame
        first_pos.header.stamp = rospy.Time.now()
        first_pos.pose = deepcopy(center_pose)
        first_pos.pose.position.y += 0.1
        first_pos.pose.position.z += 0
        
        arm.set_start_state_to_current_state()
        arm.set_pose_target(first_pos, end_effector_link)

        traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(1)

        rospy.loginfo("Drawing the circle...")

        waypoints = []

        #waypoints.append(deepcopy(first_pos))

        for i in np.arange(0, 6*math.pi, 0.05):
            wpose = deepcopy(center_pose)
            wpose.position.y += 0.1*math.cos(i)
            wpose.position.z += 0.1*math.sin(i)
            waypoints.append(deepcopy(wpose))

        fraction = 0.0
        maxtries = 100
        attempts = 0

        arm.set_start_state_to_current_state()

        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after" + str(attempts) + " attempts...")
            
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm...")
            arm.execute(plan,wait=True)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planing failed with only" + str(fraction) + "success after" + str(maxtries) + "attemps.")

        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass

