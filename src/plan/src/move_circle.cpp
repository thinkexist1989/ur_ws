
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
// #include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <Eigen/Core>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/String.h>
#include "MyRobot.h"

int main(int argc,char** argv)
{
    
    ros::init(argc, argv, "move_circle");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::string PLANNING_GROUP="manipulator";
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    int loop_rate = 30;
    ros::Rate rate( loop_rate );
    //robot_state::RobotState current_state = *group.getCurrentState();
    //group.setGoalTolerance(0.001);
    //group.setGoalJointTolerance(0.5);
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    control_msgs::FollowJointTrajectoryActionGoal g;
    
    //client_servoj_ = new Client("pos_based_pos_traj_controller/follow_joint_trajectory",true);
    int t=0;
    
    
    geometry_msgs::Pose target_pose=group.getCurrentPose().pose;
    //圆弧轨迹规划
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    double centerA=target_pose.position.x;
    double centerB=target_pose.position.y;
    double radius=0.2;//半径
    //计算圆弧轨迹
    centerA=target_pose.position.x;
    centerB=target_pose.position.y;

        for (double th=0;th<6.28;th=th+0.05)
        {
            target_pose.position.x=centerA+radius*cos(th);
            target_pose.position.y=centerB+radius*sin(th);
            waypoints.push_back(target_pose);   
        }  
        //得到工作空间的位置，反解得到关节位置，然后下发，就可以
        
        waypoints.push_back(target_pose);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold=0.0;
        const double eef_step=0.01;
        double fraction=0.0;
        int maxtries=500;//最大尝试次数
        int attempts=0;//已经尝试次数
    
    
        //group.setStartStateToCurrentState();
        
            //waypoints.push_back(target_pose);
           
            while (fraction<1.0 && attempts<maxtries)
         {
        /* code for loop body */
            fraction=group.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
            attempts++;

            if(attempts % 10 ==0)
                ROS_INFO("Still trying after %d attempts...",attempts);
         }
        
        while(ros::ok)
        {
            ros::spinOnce();
            group.setStartStateToCurrentState();
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            group.execute(plan);
            //group.move();
        
            rate.sleep();
            sleep(5.0);
        
        }
        
       
    return 0;

}

