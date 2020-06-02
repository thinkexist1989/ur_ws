#include "robot_motion/RobotMotion.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motiontest");
    ros::NodeHandle nh;

    RobotMotion ur5e(nh);

    //init joint_name
    std::vector<std::string> joint_names;
    joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("elbow_joint");

    joint_names.push_back("wrist_1_joint");
    joint_names.push_back("wrist_2_joint");
    joint_names.push_back("wrist_3_joint");

    ur5e.setJointNames(joint_names);
    ur5e.init();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
        ;

    ros::shutdown();

    return 0;
}