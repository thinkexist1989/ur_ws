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

    ur5e.MoveJ(ur5e.defaultPose["up"], 6.0, true);

    // ros::Duration(2.0).sleep();

    ur5e.MoveJ(ur5e.defaultPose["imp"], 6.0, true);

    ros::Duration(2.0).sleep();

    ur5e.wrenchCalibration = ur5e.currentEndWrench;
    // ur5e._jntTrajPtr->cancelAllGoals();

    double k = 200.0;

    KDL::Frame curPose = ur5e.currentEndPose;

    while (ros::ok())
    {
        /* code */
        double f = ur5e.currentEndWrench.force.z();
        double delta_z = f / k;
        ROS_INFO("force is: %f", f);
        // if (f < 0.2)
        //     continue;
        KDL::Frame pose = curPose;
        pose.p.z(curPose.p.z() - delta_z);
        ur5e._jntTrajPtr->cancelAllGoals();
        ur5e.MoveJ(pose, 0.2, ur5e.currentJntStates, false);
    }

    ros::shutdown();

    return 0;
}
