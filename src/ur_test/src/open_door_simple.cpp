#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <RobotMotion.hpp>
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

int main(int argc, char **argv)
{

    float radius = 0.3; //旋转半径
    float angle = 45.0; //旋转角度
    float sec = 5;      // 完成时间

    bool br = false;
    bool ba = false;
    bool bt = false;

    for (int i = 1; i < argc; i++)
    {
        if (boost::equals(argv[i - 1], "-r"))
        {
            radius = boost::lexical_cast<float>(argv[i]);
            std::cout << "radius is set to " << radius << std::endl;
            br = true;
        }
        else if (boost::equals(argv[i - 1], "-a"))
        {
            angle = boost::lexical_cast<float>(argv[i]);
            std::cout << "angle is set to" << angle << std::endl;
            ba = true;
        }
        else if (boost::equals(argv[i - 1], "-t"))
        {
            sec = boost::lexical_cast<float>(argv[i]);
            std::cout << "time is set to" << sec << std::endl;
            bt = true;
        }
    }

    if (!br)
        std::cout << "radius is set to default: " << radius << std::endl;
    if (!ba)
        std::cout << "angle is set to default: " << angle << std::endl;
    if (!bt)
        std::cout << "time is set to default: " << sec << std::endl;

    ros::init(argc, argv, "realtime_move");
    ros::NodeHandle nh;

    RobotMotion ur5e(nh);

    ur5e.init();

    ros::AsyncSpinner spinner(0);
    spinner.start();

    // ur5e.MoveJ(ur5e.defaultPose["up"], 6.0, true);

    // ros::Duration(2.0).sleep();

    ur5e.MoveJ(ur5e.defaultPose["write"], 6.0, true);

    KDL::Rotation M(0, -1, 0, -1, 0, 0, 0, 0, -1);
    KDL::Frame endPose;
    endPose.M = M;
    endPose.p = ur5e.currentEndPose.p;
    ur5e.MoveJ(endPose, 3.0, ur5e.currentJntStates, true);

    ros::Duration(2.0).sleep();

    ur5e.wrenchCalibration = ur5e.currentEndWrench;

    KDL::Vector k(500.0, 500.0, 500.0);

    // KDL::Frame curPose = ur5e.currentEndPose;

    KDL::Frame startPose = ur5e.currentEndPose;

    std::vector<KDL::Frame> endPoseVec;
    std::vector<double> times;

    int step = 100;

    for (int i = 0; i < step; i++)
    {
        KDL::Vector p = startPose.p;
        float alpha = angle / 180.0 * M_PI / step * i;
        p.y(p.y() + radius * (1 - cos(alpha)));
        p.z(p.z() - radius * sin(alpha));

        KDL::Vector vx = KDL::Vector(0, -cos(alpha), -sin(alpha));
        KDL::Vector vy = KDL::Vector(-1, 0, 0);
        KDL::Vector vz = KDL::Vector(0, sin(alpha), -cos(alpha));
        KDL::Rotation M = KDL::Rotation(vx, vy, vz);

        KDL::Frame pose;
        pose.p = p;
        pose.M = M;

        times.push_back((float)i / step * sec);
        endPoseVec.push_back(pose);
    }

    ur5e.MoveJ(endPoseVec, times, ur5e.currentJntStates, true);
    // ur5e.MoveJ(pose, 0.01, ur5e.currentJntStates, true);

    ros::shutdown();

    return 0;
}
