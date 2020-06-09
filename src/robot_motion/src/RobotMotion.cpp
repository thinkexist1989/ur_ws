#include "RobotMotion.hpp"

/**
 * @brief Construct a new Robot Motion:: Robot Motion object
 * 
 * @param nh_ 节点句柄
 * @param chainStart_ chain开始位置，默认为base_link
 * @param chainEnd_ chain末端位置，默认为tool0
 * @param timeout_ trac_ik解算时间
 * @param urdf_param_ urdf参数
 */
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

    KDL::JntArray up(6);                                             // up 位置
    up.data << 0, -M_PI_2, 0, -M_PI_2, 0, 0;                         //
    KDL::JntArray home(6);                                           // home 位置
    home.data << 0, 0, 0, 0, 0, 0;                                   //
    KDL::JntArray draw(6);                                           // draw 位置
    draw.data << -0.29629, -2.04182, -1.863, -0.855, 1.53311, 0.090; //
    KDL::JntArray write(6);                                          // write 位置
    write.data << 0.06360, -2.07364, 2.1302, -3.1982, -1.6344, 0.0;  //
    KDL::JntArray imp(6);
    imp.data << 0.041332, -2.022290, 2.367930, -1.916486, -1.570778, 0.041312; // impendance control 位置

    defaultPose.insert({"up", up});
    defaultPose.insert({"home", home});
    defaultPose.insert({"draw", draw});
    defaultPose.insert({"write", write});
    defaultPose.insert({"imp", imp});

    wrenchCalibration.force = KDL::Vector(0, 0, 0); //力传感器矫正
    wrenchCalibration.torque = KDL::Vector(0, 0, 0);
}

/**
 * @brief 初始化
 * 
 */
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

    _jntTrajPtr.reset(new Client(JOINTTRAJECTORY));
    ROS_INFO("Waiting for follow_joint_trajectory_action server to start...");
    if (_jntTrajPtr->waitForServer(ros::Duration(1.0)))
    {
        ROS_INFO("Server connected");
    }
    else
    {
        ROS_ERROR("No Server connected");
    }
}

/**
 * @brief 设置关节名称
 * 
 * @param joint_names_ 从URDF第一个活动关节开始，按顺序输入关节名称
 */
void RobotMotion::setJointNames(std::vector<std::string> joint_names_)
{
    _joint_names.clear();
    _joint_names = joint_names_;

    ROS_INFO("Joint names have been changed to %d", _joint_names.size());
}

/**
 * @brief 关节状态话题/joint_states回调函数
 * 
 * @param msg JointState类型消息
 */
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

    // std::map<std::string, int> map; // 临时存储未对应的关节名

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

    double roll, pitch, yaw;
    currentEndPose.M.GetRPY(roll, pitch, yaw);

    // ROS_INFO("Current End Pose is => x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f", currentEndPose.p.x(), currentEndPose.p.y(), currentEndPose.p.z(), roll, pitch, yaw);
    // ROS_INFO("j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f", currentJntStates(0), currentJntStates(1), currentJntStates(2), currentJntStates(3), currentJntStates(4), currentJntStates(5));

    // double roll, pitch, yaw;
    // currentEndPose.M.GetRPY(roll, pitch, yaw);
    // ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    // ROS_INFO("x: %f, y: %f, z: %f", currentEndPose.p.data[0], currentEndPose.p.data[1], currentEndPose.p.data[2]);
}

/**
 * @brief 六维力传感器数据话题/wrench接收回调函数
 * 
 * @param msg WrenchStamped消息
 */
void RobotMotion::subWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg) //callback funtion of /wrench
{
    KDL::Wrench wrench_;
    // wrench_.force = KDL::Vector(msg->wrench.force.z, -msg->wrench.force.x, -msg->wrench.force.y);
    // wrench_.torque = KDL::Vector(msg->wrench.torque.z, -msg->wrench.torque.x, -msg->wrench.torque.y);
    wrench_.force = KDL::Vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    wrench_.torque = KDL::Vector(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    // KDL::Rotation wrench2base = KDL::Rotation::RPY(0, 0, M_PI);
    // KDL::Rotation wrench2base = KDL::Rotation::RPY(0, -M_PI_2, 0) * KDL::Rotation::RPY(0, 0, M_PI_2);
    // KDL::Rotation wrench2base = KDL::Rotation::RPY(M_PI_2, -M_PI_2, 0);
    // currentEndWrench.force = wrench2base * wrench_.force;
    // currentEndWrench.torque = wrench2base * wrench_.torque;

    currentEndWrench = wrench_  - wrenchCalibration;

    // ROS_INFO("fx: %f, fy: %f, fz: %f, tx: %f, ty: %f, tz: %f", currentEndWrench.force.x(), currentEndWrench.force.y(), currentEndWrench.force.z(), currentEndWrench.torque.x(), currentEndWrench.torque.y(), currentEndWrench.torque.z());
}

//control_msgs/FollowJointTrajectoryGoal
// -trajectory_msgs/JointTrajectory trajectory
//    -std_msgs/Header header
//      -uint32 seq
//      -time stamp
//      -string frame_id
//    -string[] joint_names
//    -trajectory_msgs/JointTrajectoryPoint[] points
//      -float64[] positions
//      -float64[] velocities
//      -float64[] accelerations
//      -float64[] effort
//      -duration time_from_start
// -control_msgs/JointTolerance[] path_tolerance
//    -string name
//    -float64 position
//    -float64 velocity
//    -float64 acceleration
//- control_msgs/JointTolerance[] goal_tolerance
//    -string name
//    -float64 position
//    -float64 velocity
//    -float64 acceleration
// -duration goal_time_tolerance

/**
 * @brief 关节空间运动单个位置
 * 
 * @param jntArr 关节位置
 * @param time_from_start 运行时间
 * @param waited 是否阻塞等待完成
 */
void RobotMotion::MoveJ(KDL::JntArray &jntArr, double time_from_start, bool waited)
{
    control_msgs::FollowJointTrajectoryGoal g;
    trajectory_msgs::JointTrajectoryPoint p;
    for (int i = 0; i < _nrOfJoints; i++)
    {
        p.positions.push_back(jntArr(i));
        // p.velocities.push_back(0);
    }
    p.time_from_start = ros::Duration(time_from_start);
    g.trajectory.points.push_back(p);
    g.trajectory.header.stamp = ros::Time::now();
    g.trajectory.joint_names = _joint_names;

    if (waited)
        _jntTrajPtr->sendGoalAndWait(g);
    else
        _jntTrajPtr->sendGoal(g);
}

/**
 * @brief 关节空间运动多个位置
 * 
 * @param jntArrVec 多个关节位置
 * @param time_from_start 每个位置的运行时间 
 * @param waited 是否阻塞等待完成
 */
void RobotMotion::MoveJ(std::vector<KDL::JntArray> &jntArrVec, std::vector<double> &time_from_start, bool waited)
{
    if (jntArrVec.size() != time_from_start.size())
    {
        ROS_ERROR("ERROR::ACTION::GOAL SIZE IS NOT EQUAL");
        return;
    }

    control_msgs::FollowJointTrajectoryGoal g; //action goal

    for (int i = 0; i < jntArrVec.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint p;
        for (int j = 0; j < _nrOfJoints; j++)
        {
            p.positions.push_back(jntArrVec[i](j));
            // p.velocities.push_back(0);
        }
        p.time_from_start = ros::Duration(time_from_start[i]);

        g.trajectory.points.push_back(p);
    }

    g.trajectory.header.stamp = ros::Time::now();
    g.trajectory.joint_names = _joint_names;

    if (waited)
        _jntTrajPtr->sendGoalAndWait(g);
    else
        _jntTrajPtr->sendGoal(g);
}

void RobotMotion::MoveJ(KDL::Frame &endPose, double time_from_start, KDL::JntArray &jntInit, bool waited)
{
    KDL::JntArray jntArr;
    int res = _tracIkSolverPtr->CartToJnt(jntInit, endPose, jntArr);
    if (res < 0)
    {
        ROS_ERROR("ERROR::TRACK_IK::SOLVE_FAILED");
        return;
    }

    MoveJ(jntArr, time_from_start, waited);
}