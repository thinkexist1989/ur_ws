#include "RobotMotion.hpp"

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

    ros::AsyncSpinner spinner(0);
    spinner.start();

    KDL::JntArray joint_start(6);
    joint_start.data << -0.29629, -2.04182, -1.863, -0.855, 1.53311, 0.090;
    std::vector<KDL::JntArray> jntArrVec;
    std::vector<double> time_from_start;
    jntArrVec.push_back(joint_start);
    time_from_start.push_back(6.0);
    ur5e.MoveJ(jntArrVec, time_from_start, true);

    // ros::Duration(2.0).sleep();

    KDL::JntArray joint2(6);
    joint2.data << 0.29629, -2.04182, -1.863, -0.855, 1.53311, 0.090;
    ur5e.MoveJ(joint2, 3.0, true);

    while (ros::ok())
        ;

    ros::shutdown();

    return 0;
}