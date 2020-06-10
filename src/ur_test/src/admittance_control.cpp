#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <RobotMotion.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realtime_move");
    ros::NodeHandle nh;

    RobotMotion ur5e(nh);

    ur5e.init();

    ros::AsyncSpinner spinner(0);
    spinner.start();

    // ur5e.MoveJ(ur5e.defaultPose["up"], 6.0, true);

    // ros::Duration(2.0).sleep();

    ur5e.MoveJ(ur5e.defaultPose["imp"], 6.0, true);

    ros::Duration(2.0).sleep();

    ur5e.wrenchCalibration = ur5e.currentEndWrench;
    // ur5e._jntTrajPtr->cancelAllGoals();

    // double kx = 200.0;
    // double ky = 200.0;
    // double kz = 200.0;

    //Z向单自由度阻抗测试的情况 kz=200.0, MoveJ time_from_start=0.2 效果不错
    KDL::Vector k(500.0, 500.0, 500.0);

    // KDL::Frame curPose = ur5e.currentEndPose;
    ros::Rate rate(1000);

    while (ros::ok())
    {
        /* code */
        KDL::Vector f = ur5e.currentEndPose.M.Inverse() * ur5e.currentEndWrench.force;
        double delta_x = f.x() / k.x();
        double delta_y = f.y() / k.y();
        double delta_z = f.z() / k.z();
        // double fz = ur5e.currentEndWrench.force.z();
        // double delta_z = fz / k;
        ROS_INFO("x is: %f, y is: %f, z is: %f", f.x(), f.y(), f.z());
        KDL::Frame pose = ur5e.currentEndPose;
        pose.p.z(pose.p.z() + delta_z);
        pose.p.x(pose.p.x() + delta_x);
        pose.p.y(pose.p.y() + delta_y);

        if (ur5e._jntTrajPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ;
        }
        else
        {
            ur5e._jntTrajPtr->cancelAllGoals();
        }

        // ur5e._jntTrajPtr->stopTrackingGoal();
        // ur5e._jntTrajPtr->cancelAllGoals();
        ur5e.MoveJ(pose, 0.4, ur5e.currentJntStates, false);

        // rate.sleep();
    }

    ros::shutdown();

    return 0;
}
