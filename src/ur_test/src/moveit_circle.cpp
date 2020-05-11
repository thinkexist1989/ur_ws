#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv,"move_group_tutorials");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);

    spinner.start();

    // 确定规划组，在moveit中定义的规划组名字
    static const std::string PlANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PlANNING_GROUP);
    
    //PlanningSceneInterface用于在场景中移除、添加碰撞物体
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //TODO:不知道干啥的
    const robot_state::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup(PlANNING_GROUP);

    //RViz
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    //Remote control，可以利用RViz执行脚本
    visual_tools.loadRemoteControl();

    //RViz提供了多种marker， text, cylinders, spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE,rvt::XLARGE);

    visual_tools.trigger();

    ROS_INFO_NAMED("tutorials", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO_NAMED("tutorials", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    visual_tools.prompt("Press 'next' to start");

    //Planning to a Pose goal
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    target_pose1.position.x = 0.7;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.7;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorials", "Moving to first point %s", success ? "SUCCESS" : "FAILED");

    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

    visual_tools.trigger();

    move_group.move();

    visual_tools.prompt("Press 'next' to draw circle...");
    // std::cout << "....continued." << std::endl;
    ros::shutdown();
    
    return 0;
}