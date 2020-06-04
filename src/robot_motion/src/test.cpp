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

    // KDL::JntArray joint_start(6);
    // joint_start.data << -0.29629, -2.04182, -1.863, -0.855, 1.53311, 0.090;
    // std::vector<KDL::JntArray> jntArrVec;
    // std::vector<double> time_from_start;
    // jntArrVec.push_back(joint_start);
    // time_from_start.push_back(6.0);
    // ur5e.MoveJ(jntArrVec, time_from_start, true);
    // ros::Duration(2.0).sleep();

    // ur5e.MoveJ(ur5e.defaultPose["up"], 3.0, true);

    KDL::Vector p(0.2775, 0.1449, 0.6124);
    KDL::Rotation m = KDL::Rotation::RPY(-M_PI_2, 0, -M_PI_2);
    KDL::Frame f(m, p);
    ur5e.MoveJ(f, 6.0, ur5e.currentJntStates);

    ros::Rate rate(10);

    while (ros::ok())
    {
        double roll, pitch, yaw;
        ur5e.currentEndPose.M.GetRPY(roll, pitch, yaw);

        ROS_INFO("Current End Pose is => x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f", ur5e.currentEndPose.p.x(), ur5e.currentEndPose.p.y(), ur5e.currentEndPose.p.z(), roll, pitch, yaw);
        ROS_INFO("j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f", ur5e.currentJntStates(0), ur5e.currentJntStates(1), ur5e.currentJntStates(2), ur5e.currentJntStates(3), ur5e.currentJntStates(4), ur5e.currentJntStates(5));

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}