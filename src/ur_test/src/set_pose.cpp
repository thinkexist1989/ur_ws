#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <RobotMotion.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_pose");
    ros::NodeHandle nh;

    RobotMotion ur5e(nh);

    ur5e.init();

    ros::AsyncSpinner spinner(0);
    spinner.start();

    std::cout << "\033[1m\033[32m ************** SET UR5e END POSE ************** \033[0m" << std::endl;
    if (argc < 2)
    {
        std::cout << "Too many or too few arguments.\n"
                  << "You should use `set_pose <pose-id>` to set UR5e to desired pose. \n"
                  << "<pose-id> may be \"up\" \"home\" \"draw\" \"write\" \"imp\" "
                  << std::endl;
        return -1;
    }

    KDL::JntArray pose = ur5e.defaultPose[argv[1]];

    if (pose.data.size() == 0)
    {
        std::cout << "<pose-id> fault!" << std::endl;
    }

    std::cout << "Set UR5e end pose to "
              << "\033[1m\033[31m" << argv[1] << std::endl;

    ur5e.MoveJ(pose, 5.0, true);

    double roll, pitch, yaw;
    ur5e.currentEndPose.M.GetRPY(roll, pitch, yaw);
    std::cout << "\033[1m\033[33m"
              << "Current end postion is "
              << "\033[1m\033[33m"
              << " X: " << ur5e.currentEndPose.p.data[0] << " Y: " << ur5e.currentEndPose.p.data[1] << " Z: " << ur5e.currentEndPose.p.data[2] << std::endl;
    std::cout << "Current end oritation is "
              << "\033[1m\033[33m"
              << " R: " << roll << " P: " << pitch << " Y: " << yaw << std::endl;
    Eigen::Matrix3f mat;
    mat << ur5e.currentEndPose.M.data[0], ur5e.currentEndPose.M.data[1], ur5e.currentEndPose.M.data[2],
        ur5e.currentEndPose.M.data[3], ur5e.currentEndPose.M.data[4], ur5e.currentEndPose.M.data[5],
        ur5e.currentEndPose.M.data[6], ur5e.currentEndPose.M.data[7], ur5e.currentEndPose.M.data[8];
    std::cout << "Current end rotation Matrix is " << mat << std::endl;

    ros::shutdown();

    return 0;
}
