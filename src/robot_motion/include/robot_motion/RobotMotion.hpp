#ifndef ROBOTMOTION_H
#define ROBOTMOTION_H

#include <ros/ros.h>                                  //ros相关头文件
#include <trac_ik/trac_ik.hpp>                        //trac_ik头文件
#include <kdl/chainiksolverpos_nr_jl.hpp>             //kdl ik solver pos_NR_JL
#include <eigen3/Eigen/Eigen>                         // Eigen3头文件
#include <actionlib/client/simple_action_client.h>    // SimpleActionClient头文件
#include <control_msgs/FollowJointTrajectoryAction.h> // follow joint trajectory action头文件
#include <trajectory_msgs/JointTrajectory.h>          //follow_joint_trajectory_action_goal 需要的头文件
#include <trajectory_msgs/JointTrajectoryPoint.h>     //JointTrajectory头文件
#include <sensor_msgs/JointState.h>                   // sensor_msgs::JointState头文件
#include <geometry_msgs/WrenchStamped.h>              //geometry_msgs::WrenchStamped头文件
#include <vector>
#include <algorithm>

#define JOINT_STATES "/joint_states"
#define WRENCH "/wrench"
#define JOINTTRAJECTORY "/scaled_pos_traj_controller/follow_joint_trajectory"

class RobotMotion
{
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client; //定义action客户端
public:
    RobotMotion() = default;                                                                                                                                                                                                  // 默认构造函数 c++11特性
    RobotMotion(ros::NodeHandle nh_, std::string chainStart_ = std::string("base_link"), std::string chainEnd_ = std::string("tool0"), double timeout_ = 0.005, std::string urdf_param_ = std::string("/robot_description")); //RobotMotion构造函数

    void setJointNames(std::vector<std::string> joint_names_); // 设置_joint_names

    void init();

    ros::Subscriber subJntState; // subscribe /joint_states
    ros::Subscriber subWrench;   // subscribe /wrench

    KDL::JntArray currentJntStates;  // current joint states
    KDL::JntArray currentJntSpeeds;  // current joint speeds
    KDL::JntArray currentJntEfforts; // current joint efforts

    KDL::Frame currentEndPose; // current chainEnd pose

    KDL::Wrench currentEndWrench;  // current end Wrench
    KDL::Wrench wrenchCalibration; // wrench calibration

    void MoveJ(std::vector<KDL::JntArray> &jntArrVec, std::vector<double> &times); // MoveJ 关节空间移动

private:
    ros::NodeHandle _nh;                                            // ROS NodeHandler
    std::unique_ptr<TRAC_IK::TRAC_IK> _tracIkSolverPtr;             // TRAC_IK Solver Pointer
    KDL::Chain _chain;                                              //KDL chain
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> _fkSolverPtr;  // KDL FK Solver Pointer
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> _kdlVelIkSolverPtr; //KDL PseudoInverse Velocity Solver
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> _kdlIkSolverPtr;   // KDL IK Sovler Pointer

    std::unique_ptr<Client> _jntTrajPtr; // Follow Joint Trajectory Action Client Pointer

    std::string _chainStart; // chain start default is "base_link"
    std::string _chainEnd;   // chain end default is "tool0"
    std::string _urdf_param; // urdf description default is "/robot_description"
    double _timeout;         // max solve time default is 0.005
    double _eps;             // trac_ik solver needed
    KDL::JntArray _ll;       //low limit of n joints
    KDL::JntArray _ul;       // up limit of n joints
    unsigned int _maxiter;   // Maximum iterations

    std::vector<std::string> _joint_names; // joints' name vector
    unsigned int _nrOfJoints;              // number of joints

    void subJntStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);   // callback function of /joint_states
    void subWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg); //callback funtion of /wrench
};

#endif