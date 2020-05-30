
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/PointStamped.h>
 
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "move_group_interface_tutorial");
   ros::NodeHandle node_handle; 
   ros::AsyncSpinner spinner(1);
   spinner.start();
   moveit::planning_interface::MoveGroupInterface group("manipulator");
   moveit::planning_interface::MoveItErrorCode success;
 
   //设置初始位置
   group.setStartState(*group.getCurrentState());
 
   //设置约束
   moveit_msgs::Constraints endEffector_constraints;
   moveit_msgs::OrientationConstraint ocm;
   ocm.link_name = "ee_link";//需要约束的链接
   ocm.header.frame_id = "base_link";//基坐标系
   //四元数约束
   ocm.orientation.w = 1.0;
   //欧拉角约束
   ocm.absolute_x_axis_tolerance = 0.1;
   ocm.absolute_y_axis_tolerance = 0.1;
   ocm.absolute_z_axis_tolerance = 2*3.14;
   ocm.weight = 1.0;//此限制权重
   endEffector_constraints.orientation_constraints.push_back(ocm);//加入限制列表
	
    //设置运动路径
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w=1;
    target_pose1.orientation.x=0.28;
    target_pose1.orientation.y=-0.7;
    target_pose1.orientation.z=1.0;
    
    target_pose1.position.x = 0.60;
    target_pose1.position.y = -0.379;
    target_pose1.position.z = 1.02;
    waypoints.push_back(target_pose1);
 
     geometry_msgs::Pose target_pose2;
    target_pose2.position.x = 0.70;
    target_pose2.position.y = -0.379;
    target_pose2.position.z = 1.02;
    waypoints.push_back(target_pose2);
   
    //进行运动规划
    moveit_msgs::RobotTrajectory trajectory_msg;
    //直线插补函数
    double fraction = group.computeCartesianPath( waypoints, 
                                                    0.01, // eef_step, 
						                            0.0,  // jump_threshold
						                        trajectory_msg, 
                                                endEffector_constraints,//constraints
						                            false);
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);  
	group.clearPathConstraints();					   
    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "endeffector");
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg); 
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool ItSuccess = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",ItSuccess?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_msg);
 
    //输出运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory_msg;
    //group.move();
  
        
        group.move();
        group.execute(plan);
   
 
   ros::shutdown(); 
   return 0;
}
