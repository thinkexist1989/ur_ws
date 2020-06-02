#include "robot_motion/RobotMotion.hpp"

RobotMotion::RobotMotion(ros::NodeHandle nh_, std::string chainStart_, std::string chainEnd_, double timeout_, std::string urdf_param_) : _nh(nh_),
                                                                                                                                          _chainStart(chainStart_),
                                                                                                                                          _chainEnd(chainEnd_),
                                                                                                                                          _timeout(timeout_),
                                                                                                                                          _urdf_param(urdf_param_),
                                                                                                                                          _eps(1e-5),
                                                                                                                                          _maxiter(1),
                                                                                                                                          _nrOfJoints(0)
{
    _joint_names.push_back("shoulder_pan_joint");
    _joint_names.push_back("shoulder_lift_joint");
    _joint_names.push_back("elbow_joint");
    _joint_names.push_back("wrist_1_joint");
    _joint_names.push_back("wrist_2_joint");
    _joint_names.push_back("wrist_3_joint");
}

void RobotMotion::init()
{
    if (!_nh.ok())
    {
        ROS_ERROR("ROS Node Handler is not OK!");
        return;
    }
    if (_urdf_param.empty())
    {
        ROS_ERROR("No Robot Description URDF found!");
        return;
    }

    _tracIkSolverPtr.reset(new TRAC_IK::TRAC_IK(_chainStart, _chainEnd, _urdf_param, _timeout, _eps)); // TRAC_IK solver

    bool valid = _tracIkSolverPtr->getKDLChain(_chain); // store chain in _chain

    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found!");
        return;
    }

    valid = _tracIkSolverPtr->getKDLLimits(_ll, _ul); // store joint limits

    if (!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found!");
        return;
    }

    assert(_chain.getNrOfJoints() == _ll.data.size());
    assert(_chain.getNrOfJoints() == _ul.data.size());

    ROS_INFO("Using %d joints", _chain.getNrOfJoints());

    _nrOfJoints = _chain.getNrOfJoints(); //save number of joints

    if (_nrOfJoints != _joint_names.size())
    {
        ROS_ERROR("The number of joints in chain is %d, not equal to that in joint_names %d", _nrOfJoints, _joint_names.size());
        return;
    }

    _fkSolverPtr.reset(new KDL::ChainFkSolverPos_recursive(_chain));                                                              // KDL FK Solver
    _kdlVelIkSolverPtr.reset(new KDL::ChainIkSolverVel_pinv(_chain));                                                             // KDL IK PseudoInverse vel solver
    _kdlIkSolverPtr.reset(new KDL::ChainIkSolverPos_NR_JL(_chain, _ll, _ul, *_fkSolverPtr, *_kdlVelIkSolverPtr, _maxiter, _eps)); // KDL IK Pos Solver

    subJntState = _nh.subscribe(JOINT_STATES, 1, &RobotMotion::subJntStatesCallback, this);
    subWrench = _nh.subscribe(WRENCH, 1, &RobotMotion::subWrenchCallback, this);
}

void RobotMotion::setJointNames(std::vector<std::string> joint_names_)
{
    _joint_names.clear();
    _joint_names = joint_names_;

    ROS_INFO("Joint names have been changed to %d", _joint_names.size());
}

void RobotMotion::subJntStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) // callback function of /joint_states
{
    KDL::JntArray jntStates_(6);
    KDL::JntArray jntSpeeds_(6);
    KDL::JntArray jntEfforts_(6);

    if (_nrOfJoints != msg->name.size())
    {
        ROS_ERROR("The number of joints is not equal which urdf is %d, msg is %d", _nrOfJoints, (int)msg->name.size());
        return;
    }

    std::map<std::string, int> map; // 临时存储未对应的关节名

    for (int i = 0; i < _nrOfJoints; i++)
    {
        std::vector<std::string>::iterator iter = std::find(_joint_names.begin(), _joint_names.end(), msg->name[i]);
        if (iter != _joint_names.end())
        {
            int k = iter - _joint_names.begin();
            jntStates_(k) = msg->position[i];
            jntSpeeds_(k) = msg->velocity[i];
            jntEfforts_(k) = msg->effort[i];
        }
        else
        {
            ROS_ERROR("The joint name %s is not found", msg->name[i]);
            return;
        }
    }

    currentJntStates = jntStates_;
    currentJntSpeeds = jntSpeeds_;
    currentJntEfforts = jntEfforts_;

    int res = _fkSolverPtr->JntToCart(currentJntStates, currentEndPose);

    // ROS_INFO("j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f", currentJntStates(0), currentJntStates(1), currentJntStates(2), currentJntStates(3), currentJntStates(4), currentJntStates(5));

    // double roll, pitch, yaw;
    // currentEndPose.M.GetRPY(roll, pitch, yaw);
    // ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    // ROS_INFO("x: %f, y: %f, z: %f", currentEndPose.p.data[0], currentEndPose.p.data[1], currentEndPose.p.data[2]);
}

void RobotMotion::subWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg) //callback funtion of /wrench
{
    KDL::Wrench wrench_;
    wrench_.force = KDL::Vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    wrench_.torque = KDL::Vector(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    // KDL::Rotation wrench2base = KDL::Rotation::RPY(0, 0, M_PI);
    // KDL::Rotation wrench2base = KDL::Rotation::RPY(0, -M_PI_2, 0) * KDL::Rotation::RPY(0, 0, M_PI_2);
    // KDL::Rotation wrench2base = KDL::Rotation::RPY(M_PI_2, 0, M_PI_2);
    // currentEndWrench.force = wrench2base * wrench_.force;
    // currentEndWrench.torque = wrench2base * wrench_.torque;

    currentEndWrench = wrench_;

    ROS_INFO("fx: %f, fy: %f, fz: %f, tx: %f, ty: %f, tz: %f", currentEndWrench.force.x(), currentEndWrench.force.y(), currentEndWrench.force.z(), currentEndWrench.torque.x(), currentEndWrench.torque.y(), currentEndWrench.torque.z());
}