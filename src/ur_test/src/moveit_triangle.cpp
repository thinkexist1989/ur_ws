//简单的手动控制UR机械臂运动的程序
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_triangle");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);

    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::Pose start_pose;

    start_pose.orientation.w = 1.0;
    start_pose.position.x = 0.7;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.7;
    move_group.setPoseTarget(start_pose);

    move_group.move();
    

    while(1)
    {
        std::string order;
        std::cin >> order;

        if(order == "q")
            break;
        else if(order == "a"){
            std::cout << "Press a, move left"<< std::endl;
            geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
            pose.position.y += 0.1;
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(pose);
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("move_group_triangle", "Cartesian Path %.2f%% is achieved", fraction*100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group.execute(plan);
        }
        else if(order == "s"){
            std::cout << "Press s, move down"<< std::endl;
            geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
            pose.position.z -= 0.1;
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(pose);
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("move_group_triangle", "Cartesian Path %.2f%% is achieved", fraction*100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group.execute(plan);
        }
        else if(order == "d"){
            std::cout << "Press d, move right"<< std::endl;
            geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
            pose.position.y -= 0.1;
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(pose);
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("move_group_triangle", "Cartesian Path %.2f%% is achieved", fraction*100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group.execute(plan);
        }
        else if(order == "w"){
            std::cout << "Press w, move up"<< std::endl;
            geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
            pose.position.z += 0.1;
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(pose);
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("move_group_triangle", "Cartesian Path %.2f%% is achieved", fraction*100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group.execute(plan);
        }
        

    }

    std::cout << "Program Finished!" << std::endl;

    ros::shutdown();

    return 0;


}