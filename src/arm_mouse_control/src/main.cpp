#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>

#include <RobotMotion.hpp>

// using namespace std;
// using namespace cv;

// std::vector<geometry_msgs::Pose> waypoints;

Eigen::Matrix3f trans;

RobotMotion *ur5e;
KDL::Frame startPose;

KDL::Frame currentPose;

cv::Mat dashboard;
cv::Point prePoint(0, 0);
bool draw = false;

void mouseHandler(int event, int x, int y, int flags, void *ustc) //event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
    // static Point pre_pt(-1, -1); //初始坐标
    // static Point cur_pt(-1, -1); //实时坐标
    // char temp[16];
    Eigen::Vector3f img, ur;
    img << x, y, 1;
    ur = trans * img;

    cv::Point curPoint(x, y);

    KDL::Frame f(startPose);
    f.p.y(startPose.p.y() + ur(0) / 1000.0);
    f.p.z(startPose.p.z() + ur(1) / 1000.0);

    currentPose = f;

    if (event == CV_EVENT_LBUTTONDOWN) //左键按下，机械臂向+x方向前进0.1m
    {
        // KDL::Frame f(ur5e->currentEndPose);
        // f.p.x(f.p.x() + 0.05);
        // ur5e->MoveJ(f, 0.05, ur5e->currentJntStates, true);
        draw = true;
        ROS_INFO("Left Button Down! ur: %f, %f", ur(0), ur(1));
    }
    else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON)) //左键没有按下的情况下鼠标移动的处理函数
    {
    }
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON)) //左键按下时，鼠标移动，则在图像上划矩形
    {
        cv::line(dashboard, prePoint, curPoint, cv::Scalar(0), 2, 8);
        // geometry_msgs::Pose pose;
        // pose.orientation.w = 1.0;
        // pose.position.x = 0.75;
        // pose.position.y = 0.0 + ur(0) / 1000.0;
        // pose.position.z = 0.70 + ur(1) / 1000.0;
        ROS_INFO("ur: %f, %f", ur(0), ur(1));
    }
    else if (event == CV_EVENT_LBUTTONUP) //左键松开，机械臂向-x方向退回0.1m
    {
        draw = false;
        ROS_INFO("Left Button Up! ur: %f, %f", ur(0), ur(1));
    }
    prePoint = cv::Point(x, y);
}

double distance(KDL::Frame f1, KDL::Frame f2)
{
    return sqrt(f1.p.x() * f2.p.x() + f1.p.y() * f2.p.y() + f1.p.z() * f2.p.z());
}
int main(int argc, char **argv)
{
    // org = imread("1.jpg");
    ros::init(argc, argv, "ur_mouse_go");
    ros::NodeHandle nh;
    ur5e = new RobotMotion(nh);
    ur5e->init();
    ROS_INFO("ur5e init ok~");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    trans << -1, 0, 200,
        0, -1, 100,
        0, 0, 1;

    // ur5e->MoveJ(ur5e->defaultPose["up"], 3.0, true);
    // ros::Duration(1.0).sleep();
    ur5e->MoveJ(ur5e->defaultPose["write"], 3.0, true);
    startPose = ur5e->currentEndPose;

    dashboard.create(200, 400, CV_8UC1);
    dashboard.setTo(cv::Scalar(255));

    cv::namedWindow("UR_GO");                       //定义一个窗口
    cv::setMouseCallback("UR_GO", mouseHandler, 0); //调用回调函数

    ros::Rate rate(50);
    while (ros::ok())
    {
        imshow("UR_GO", dashboard);
        if (draw)
        {
            ur5e->MoveJ(currentPose, distance(currentPose, ur5e->currentEndPose) / 2.0, ur5e->currentJntStates, true);
        }
        // rate.sleep();
        cv::waitKey(1);
    }

    ros::shutdown();

    return 0;
}
