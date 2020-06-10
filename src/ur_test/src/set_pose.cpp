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

    std::cout << "\033[1m\033[32m ************** SET UR5e END POSE ************** \033[0m" << std::endl;
    if (argc < 2)
    {
        std::cout << "Too many or too few arguments. \\
                      You should use `set_pose <pose-id>` to set UR5e to desired pose. \\
                      <pose-id> may be \"up\" \"home\" \"draw\" \"write\" \"imp\" "
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

    ros::shutdown();

    return 0;
}
