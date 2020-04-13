#!/usr/bin/env python

import rospy, sys, math
import moveit_commander
import numpy as np
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItObstaclesDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_obstacles_demo')

        scene = PlanningSceneInterface()

        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)

        self.colors = dict()

        rospy.sleep(1)

        arm = MoveGroupCommander('manipulator')

        end_effector_link = arm.get_end_effector_link()

        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        arm.allow_replanning(True)

        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        arm.set_planning_time(5)

        ground_id = 'ground'
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'

        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        rospy.sleep(1)

        arm.set_named_target('up')
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        arm.go()
        rospy.sleep(1)

        table_ground = 0.25

        ground_size = [2,2, 0.01]
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]

        ground_pose = PoseStamped()
        ground_pose.header.frame_id = reference_frame
        ground_pose.pose.position.x = 0
        ground_pose.pose.position.y = 0
        ground_pose.pose.position.z = -0.1
        ground_pose.pose.orientation.w = 1.0
        scene.add_box(ground_id, ground_pose, ground_size)

        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = -0.66
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = -0.61
        box1_pose.pose.position.y = -0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0
        scene.add_box(box1_id, box1_pose, box1_size)

        box2_pose = PoseStamped()
        box2_pose.header.frame_id = reference_frame
        box2_pose.pose.position.x = -0.59
        box2_pose.pose.position.y = 0.15
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0
        scene.add_box(box2_id, box2_pose, box2_size)

        self.setColor(ground_id, 1.0, 1.0, 1.0)
        self.setColor(table_id, 0.8, 0, 0.8, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)

        self.sendColors()

        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = -0.6
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = table_pose.pose.position.z + table_size[2] +0.01
        q = quaternion_from_euler(np.deg2rad(0), np.deg2rad(90), np.deg2rad(180))
        target_pose.pose.orientation = Quaternion(*q)
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()
        rospy.sleep(1)

        target_pose2 = PoseStamped()
        target_pose2.header.frame_id = reference_frame
        target_pose2.pose.position.x = -0.6
        target_pose2.pose.position.y = -0.25
        target_pose2.pose.position.z = table_pose.pose.position.z + table_size[2] + 0.01
        target_pose2.pose.orientation = Quaternion(*q)

        arm.set_pose_target(target_pose2, end_effector_link)
        arm.go()
        rospy.sleep(1)

        target_pose3 = PoseStamped()
        target_pose3.header.frame_id = reference_frame
        target_pose3.pose.position.x = -0.6
        target_pose3.pose.position.y = 0.25
        target_pose3.pose.position.z = table_pose.pose.position.z + table_size[2] + 0.01
        target_pose3.pose.orientation = Quaternion(*q)

        arm.set_pose_target(target_pose3, end_effector_link)
        arm.go()
        rospy.sleep(1)

        arm.set_named_target('up')
        arm.go()

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    
    def setColor(self, name, r, g, b, a = 0.9):
        color = ObjectColor()

        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        self.colors[name] = color
    
    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True

        for color in self.colors.values():
            p.object_colors.append(color)
        
        self.scene_pub.publish(p)


if __name__ == '__main__':
    try:
        MoveItObstaclesDemo()
    except KeyboardInterrupt:
        raise