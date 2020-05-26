#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>

// using namespace std;
// using namespace cv;

// 确定规划组，在moveit中定义的规划组名字
static const std::string PlANNING_GROUP = "manipulator";

moveit::planning_interface::MoveGroupInterface* move_group_ptr;

geometry_msgs::Pose currentPose;

// std::vector<geometry_msgs::Pose> waypoints;

Eigen::Matrix3f trans;

void cartesian_move_once_step(moveit::planning_interface::MoveGroupInterface* move_group_ptr,double x_step, double y_step, double z_step)
{
    geometry_msgs::Pose pose = move_group_ptr->getCurrentPose().pose;
    pose.position.x += x_step;
    pose.position.y += y_step;
    pose.position.z += z_step;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("arm_control_gui", "Cartesian Path %.2f%% is achieved.", fraction*100);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_ptr->execute(plan);
}


void cartesian_move_once(moveit::planning_interface::MoveGroupInterface* move_group_ptr,double x, double y, double z)
{
    geometry_msgs::Pose pose = move_group_ptr->getCurrentPose().pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("arm_control_gui", "Cartesian Path %.2f%% is achieved.", fraction*100);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_ptr->execute(plan);
}

void mouseHandler(int event, int x, int y, int flags, void *ustc) //event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
    // static Point pre_pt(-1, -1); //初始坐标
    // static Point cur_pt(-1, -1); //实时坐标
    // char temp[16];
    Eigen::Vector3f img, ur;
    img << x, y, 1;
    ur = trans*img;

    if (event == CV_EVENT_LBUTTONDOWN) //左键按下，机械臂向+x方向前进0.1m
    {
        geometry_msgs::Pose pose = move_group_ptr->getCurrentPose().pose;
        pose.position.x = 0.70;
        pose.position.y = 0.0 + ur(0)/1000.0;
        pose.position.z = 0.70 + ur(1)/1000.0;
        std::cout << pose.position.y << std::endl;
        std::cout << pose.position.z << std::endl;
        move_group_ptr->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        move_group_ptr->move();  

        pose.position.x = 0.75;

        move_group_ptr->setPoseTarget(pose);
        success = (move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        move_group_ptr->move();  

        // // std::vector<geometry_msgs::Pose> waypoints;
        // std::vector<geometry_msgs::Pose> waypoints;
        // // waypoints.push_back(pose);
        // pose.position.x = 0.75;
        // waypoints.push_back(pose);

        // moveit_msgs::RobotTrajectory trajectory;
        // const double jump_threshold = 0.0;
        // const double eef_step = 0.01;
        // double fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        // ROS_INFO_NAMED("arm_control_gui", "Cartesian Path %.2f%% is achieved.", fraction*100);
        // moveit::planning_interface::MoveGroupInterface::Plan plan;
        // plan.trajectory_ = trajectory;
        // move_group_ptr->execute(plan);

        // currentPose = move_group_ptr->getCurrentPose().pose;
    }
    else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON)) //左键没有按下的情况下鼠标移动的处理函数
    {
    }
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON)) //左键按下时，鼠标移动，则在图像上划矩形
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.75;
        pose.position.y = 0.0 + ur(0)/1000.0;
        pose.position.z = 0.70 + ur(1)/1000.0;
        move_group_ptr->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("arm_mouse_control", "Moving... x is: %d, y is: %d .", x, y);

        move_group_ptr->move();     

    }
    else if (event == CV_EVENT_LBUTTONUP) //左键松开，机械臂向-x方向退回0.1m
    {
        // cartesian_move_once(move_group_ptr, 0.7, currentPose.position.y, currentPose.position.z);
        geometry_msgs::Pose pose = move_group_ptr->getCurrentPose().pose;
        pose.position.x = 0.70;

        move_group_ptr->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        move_group_ptr->move();


        // std::vector<geometry_msgs::Pose> waypoints;
        // waypoints.push_back(pose); 

        // moveit_msgs::RobotTrajectory trajectory;
        // const double jump_threshold = 0.0;
        // const double eef_step = 0.01;
        // double fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        // ROS_INFO_NAMED("arm_control_gui", "Cartesian Path %.2f%% is achieved.", fraction*100);
        // moveit::planning_interface::MoveGroupInterface::Plan plan;
        // plan.trajectory_ = trajectory;
        // move_group_ptr->execute(plan);
        // waypoints.clear();
    }
}
int main(int argc, char** argv)
{
    // org = imread("1.jpg");
    cv::Mat dashboard;
    ros::init(argc, argv, "ur_mouse_go");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);

    spinner.start();

    move_group_ptr = new moveit::planning_interface::MoveGroupInterface(PlANNING_GROUP);


    const moveit::core::JointModelGroup* joint_model_group = move_group_ptr->getCurrentState()->getJointModelGroup(PlANNING_GROUP);  

    geometry_msgs::Pose start_pose;

    start_pose.orientation.w = 1.0;
    start_pose.position.x = 0.7;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.7;
    move_group_ptr->setPoseTarget(start_pose);

    move_group_ptr->move();

    currentPose = start_pose;

    trans << -1,  0, 200,
              0, -1, 100,
              0,  0,   1;

    dashboard.create(200, 400, CV_8UC1);
    dashboard.setTo(cv::Scalar(255));

    cv::namedWindow("UR_GO");                   //定义一个窗口
    cv::setMouseCallback("UR_GO", mouseHandler, 0); //调用回调函数
    imshow("UR_GO", dashboard);
    cv::waitKey(0);

    ros::shutdown();

    return 0;
}

