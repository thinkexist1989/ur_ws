
#ifndef MYROBOT_H
#define MYROBOT_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/String.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/String.h>
#include <trac_ik/trac_ik.hpp>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <math.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/robot_hw.h>
#include <force_control/forcecontrol_msg.h>
using namespace Eigen;

class MyRobot :public hardware_interface::RobotHW
{
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;//defin a client to send order 
public:
    MyRobot(ros::NodeHandle _nh, const std::string & _urdf_param, const std::string & _chain_start, const std::string & _chain_end, double _timeout,std::string UR_name_prefix);
   ~MyRobot();
    //void init_robot();//init the arm
    void move_joint_to_star(std::vector<double> start_joint);//set the arm to the start poition
    void set_robot_to_up();//set the arm to up
    void draw_circle();//draw circle input center_x,center_y,force need to do in the next day
    void move_line(std::vector<double> start_posintion,std::vector<double> goal_posintion);
    void force_update_all_state();//update the force values
    void force_control();//need to read the joints angles and the force values and ik values
    //void force_interaction();//hunman interaction with robot
    void robotiq_hand_move(float position,float vel, float force);//move hand based on force
    void catch_bottle();//catch the bottle
    void move_to_xyz(double x,double y,double z);//need ik values
    void move_joints(KDL::JntArray target_joint, double time);//plan in the joint space
    void servoj_moveto();//creat a new cilient to set order to the hardware 
    void compu_inverse_frame();//ik 
    void servoj_moveto(KDL::JntArray target, double time,bool wait_for_D);
    void draw_sin();
    void servoj_sin_test();
    void draw_circle_servioj();
    void force_interaction();
    void sub_ur_update();
    void write();
    void read();
private:

 void subJointStatesCB(sensor_msgs::JointState state)
    {
        KDL::JntArray jntArr;
        KDL::JntArray jntSpeed;
        KDL::JntArray jntCur;
        jntArr.resize(6);
        jntSpeed.resize(6);
        jntCur.resize(6);
        int n = state.name.size();
        for (int i = 0; i < 6; ++i)//joint_names_
        {
            int x = 0;
            for (; x < n; ++x)//state
            {
                if (state.name[x] == ( joint_names_[i])) {
                    jntArr(i) = state.position[x];
                    jntSpeed(i) = state.velocity[x];
                    jntCur(i) = state.effort[x];
                    break;
                }
            }

            if (x == n) {
                ROS_ERROR_STREAM("Error,  joint name : " << joint_names_[i] << " , not found.  ");
                return;
            }
        }
        current_JntArr_ = jntArr;
        current_JntSpeed = jntSpeed;
//      current_JntCur_ = jntCur;

        //fk
        p_fk_solver_->JntToCart(current_JntArr_, frame_wrist3_base_);    //frame_wrist3_base_ 为正运动学计算出的位姿
        end_point_.data[0] = frame_wrist3_base_.p.data[0];
        end_point_.data[1] = frame_wrist3_base_.p.data[1];
        end_point_.data[2] = frame_wrist3_base_.p.data[2];
        geometry_msgs::Point end_point;
        end_point.x = end_point_.x();
        end_point.y = end_point_.y();
        end_point.z = end_point_.z();
        pub_end_point_.publish(end_point);
//        if ( (frame_wrist3_base_.p - end_point_).Norm()> 0.08 )
//            ROS_WARN("bug");
        bur_sub_ = true;//标志位
        ROS_INFO("bur_sub_ = true");
    }

    void sub_wrench(geometry_msgs::WrenchStamped state)
    {
        wrench_base_.force =  KDL::Vector(state.wrench.force.x,state.wrench.force.y,state.wrench.force.z );
        wrench_base_.torque =  KDL::Vector(state.wrench.torque.x,state.wrench.torque.y,state.wrench.torque.z );
        KDL::Rotation R_base2base_link = KDL::Rotation::RPY(0,0,M_PI);
        wrench_base_.force = R_base2base_link * wrench_base_.force;
        wrench_base_.torque = R_base2base_link * wrench_base_.torque;
        wrench_base_ = wrench_base_ - wrench_repair_;
//        ROS_INFO_STREAM("wrench_repaired: "<<wrench_base_.force.data[0]<<" "
//                                           <<wrench_base_.force.data[1]<<" "
//                                           <<wrench_base_.force.data[2]<<" ");
//        geometry_msgs::WrenchStamped ur_wrench = state;
//        geometry_msgs::Vector3 ur_wrench_tem = ur_wrench.wrench.force;
//        ur_wrench_now = KDL::Vector(ur_wrench_tem.x, ur_wrench_tem.y, ur_wrench_tem.z );
        bsub_wrench_ = true;
//        if( (ur_wrench_now - start_wrench).Norm() > 20 )
//        {
//            wrench_ok = false;
//            ROS_FATAL_STREAM("the wrench is to big "<< (ur_wrench_now - start_wrench).Norm() );
//        } else wrench_ok = true;
//        ROS_INFO_STREAM("wrench "<< (ur_wrench_now - start_wrench).Norm() <<" wrench now  "<<ur_wrench_now.z());
    }
    
private:
    /* data */
    ros::NodeHandle nh_;
    TRAC_IK::TRAC_IK *p_tracik_solver_;
    KDL::ChainFkSolverPos_recursive *p_fk_solver_;
    Client *client_;
    Client *client_servoj_;
    std::vector<std::string> joint_names_;
    KDL::JntArray current_JntArr_;
    KDL::JntArray current_JntSpeed;
    KDL::Frame frame_wrist3_base_;//fk result
    KDL::Vector end_point_;

    bool bsub_;
    bool bur_sub_;
    bool bstart_cal_;

    //ur5e
    ros::Subscriber sub_ ;
    ros::Subscriber sub_markers_;
    ros::Publisher pub_end_point_;
    ros::Subscriber ur_wrench_;

    KDL::Vector start_wrench;
    bool bsub_wrench_;
    KDL::Vector ur_wrench_now;
    bool wrench_ok = true;
    
     // wrench
    KDL::Wrench wrench_base_;
    KDL::Wrench wrench_repair_;
    
};

MyRobot::MyRobot(ros::NodeHandle _nh, const std::string & _urdf_param, const std::string & _chain_start, const std::string & _chain_end, double _timeout,std::string UR_name_prefix)
{
    double eps=1e-5;
    nh_=_nh;
    p_tracik_solver_ = new TRAC_IK::TRAC_IK(_chain_start, _chain_end, _urdf_param, _timeout, eps); //反解
    KDL::Chain chain;
    bool valid = p_tracik_solver_->getKDLChain(chain);
    p_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain);  //正解
    //TRAC_IK::TRAC_IK tracik_solver(_chain_start, _chain_end, _urdf_param, _timeout, eps);
    
    if (!valid) 
    {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }

    //init 
        joint_names_.push_back("shoulder_pan_joint");
        joint_names_.push_back("shoulder_lift_joint");
        joint_names_.push_back("elbow_joint");
        joint_names_.push_back("wrist_1_joint");
        joint_names_.push_back("wrist_2_joint");
        joint_names_.push_back("wrist_3_joint");

        current_JntArr_.resize(6);
        for(int i=0;i<6;++i)
        {
            if (i==1||i==3)
                current_JntArr_(i) = -M_PI/2;
            else if(i==0)
                current_JntArr_(i) = M_PI;
            else
                current_JntArr_(i) = 0;
        }
        bsub_ = false;
        bstart_cal_ = false;
    //sub
        sub_=nh_.subscribe("/joint_states", 1, &MyRobot::subJointStatesCB, this);
        pub_end_point_ = nh_.advertise<geometry_msgs::Point>("end_point",1);

       // sub_markers_ = nh_.subscribe("vis_markers", 1, &MyRobot::subMotionCaptureCB, this);
        //sub_myo_hand_ = nh_.subscribe("/myo_raw/myo_emg",1,&UR::sub_myo_handCall,this);
        ur_wrench_ = nh_.subscribe(UR_name_prefix+"/wrench", 1,&MyRobot::sub_wrench,this);
        //ros::spinOnce();
        ROS_INFO("sub done");
        ROS_INFO("M_PI %f ", M_PI);
        client_servoj_ = new Client("/scaled_pos_traj_controller/follow_joint_trajectory",true);
        ROS_INFO("wait for servoj client");
        client_servoj_->waitForServer(ros::Duration());
        ROS_INFO("servoj clent connect!");

        // //action client
        // client_ = new Client(UR_name_prefix+"/pos_traj_controller/follow_joint_trajectory", true);
        // ROS_INFO("wait for action client");
        // client_->waitForServer(ros::Duration());
        // ROS_INFO("action clent connect!");

        wrench_repair_.force = KDL::Vector(0,0,0);
        wrench_repair_.torque = KDL::Vector(0,0,0);

} 
MyRobot::~MyRobot()
{
    delete p_tracik_solver_;
    delete client_;
    delete client_servoj_;

}

void MyRobot::move_joint_to_star(std::vector<double> start_joint)//移动机械臂到起始位置
{
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setGoalTolerance(0.001);
    group.setGoalJointTolerance(0.5);
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setJointValueTarget(start_joint);

    group.move();   
    ROS_INFO("move to start");
}

void MyRobot::write()
{

}

void MyRobot::read()
{

}

void MyRobot::sub_ur_update()
    {
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while (!bur_sub_);
    }

void MyRobot::draw_circle()//draw circle
{
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    geometry_msgs::Pose target_pose=group.getCurrentPose().pose;
    //圆弧轨迹规划
    std::vector<geometry_msgs::Pose> waypoints;
    KDL::JntArray waypoints1;
    waypoints.push_back(target_pose);
   
    double centerA=target_pose.position.x;
    double centerB=target_pose.position.y;
    double radius=0.2;//半径
    //计算圆弧轨迹
    centerA=target_pose.position.x;
    centerB=target_pose.position.y;

        for (double th=0;th<6.28;th=th+0.05)
        {
            target_pose.position.x=centerA+radius*cos(th);
            target_pose.position.y=centerB+radius*sin(th);
            waypoints.push_back(target_pose);   
        }  
        //得到工作空间的位置，反解得到关节位置，然后下发，就可以
        
        waypoints.push_back(target_pose);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold=0.0;
        const double eef_step=0.01;
        double fraction=0.0;
        int maxtries=500;//最大尝试次数
        int attempts=0;//已经尝试次数
    
        while (fraction<1.0 && attempts<maxtries)
         {
        /* code for loop body */
            fraction=group.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
            attempts++;

            if(attempts % 10 ==0)
                ROS_INFO("Still trying after %d attempts...",attempts);
         }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            //group.execute(plan);
            KDL::JntArray joint_start;
            for (int i=0;i<6;i++)
            {
                joint_start(i)=0;
            }
            this->servoj_moveto(joint_start,6,true);

}

void MyRobot::set_robot_to_up()
{
    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface plan_group("manipulator");
    // Create a published for the arm plan visualization

    // Get the current RobotState, which will be used to set the arm
    // to one of the predefined group states, in this case home
    robot_state::RobotState current_state = *plan_group.getCurrentState();

    // We set the state values for this robot state to the predefined
    // group state values
    current_state.setToDefaultValues(current_state.getJointModelGroup("manipulator"), "up");

    // We set the current state values to the target values
    plan_group.setJointValueTarget(current_state);


    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
    if (plan_group.plan(goal_plan))
    {
        
        sleep(5.0);
        plan_group.move();
        plan_group.execute(goal_plan);
    }

}

void MyRobot::move_joints(KDL::JntArray target_joint, double time)
{
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(bur_sub_==false);//跟新关节角度信息，一直更新

        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");

        for(int x=0;x<6;++x)
        {
            p0.positions.push_back(current_JntArr_(x));
            p0.velocities.push_back(0);
        }
        p0.time_from_start = ros::Duration(0);
        g.trajectory.points.push_back(p0);

        for(int i=0;i<6;++i)
        {
            p1.positions.push_back(target_joint(i));
            p1.velocities.push_back(0);
        }
        
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_servoj_->sendGoal(g);
        client_servoj_->waitForResult(ros::Duration());

}

 void MyRobot::servoj_moveto(KDL::JntArray target, double time,bool wait_for_D) //test success!
 {
        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");


        for(int x=0;x<6;++x)
        {
            p1.positions.push_back(target(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_servoj_->sendGoal(g);

        if(wait_for_D)
        {
            ros::Duration(time).sleep();
        }

     
}

void MyRobot::draw_sin()
    {
        int dir =1;
        KDL::Chain chain;
        bool valid = p_tracik_solver_->getKDLChain(chain);//get the robot chain

        //start jntArr
        uint nbr = chain.getNrOfJoints();//return total number of segments
        KDL::JntArray p(nbr);
        p = current_JntArr_;
//    for(int i=0;i<20;++i)
//    {
//      /*if(bsub_){
//        p = current_JntArr_;
//        break;
//       }
//      else
//      {
//         ros::Duration(0.5).sleep();
//         ROS_INFO_STREAM("loop"<<i);
//      }*/
//   }
        for(unsigned int i=0;i<nbr;++i)
        {
            ROS_INFO_STREAM("name: "<<chain.getSegment(i).getJoint().getName()<<"   pos: "<<p(i)<<"\n");
        }

        //ee pose
        KDL::Frame end_effector_pose;
        end_effector_pose.M.data[0] = 1;//初始化旋转矩阵
        end_effector_pose.M.data[1] = 0;
        end_effector_pose.M.data[2] = 0;
        end_effector_pose.M.data[3] = 0;
        end_effector_pose.M.data[4] = 1;
        end_effector_pose.M.data[5] = 0;
        end_effector_pose.M.data[6] = 0;
        end_effector_pose.M.data[7] = 0;
        end_effector_pose.M.data[8] = 1;

        //cal
        std::vector<KDL::JntArray>  vResult;
        KDL::JntArray result;
        int i=0, range=50;
        for(i=-range;i<range;++i)
        {
            ROS_INFO("Pt %d ", i);
            end_effector_pose.p.data[0] = dir*(i*M_PI/(30*range)+0.1);//mm
            end_effector_pose.p.data[1] = 0.325;
            end_effector_pose.p.data[2] = 0.05*sin(i*M_PI/range)+0.435;
            p_tracik_solver_->CartToJnt(p, end_effector_pose, result);

            p = result;
            vResult.push_back(result);
        }
        //reverse
        for(i=range;i>-range;--i)
        {
            //     ROS_INFO("Pt %d ", i);
            end_effector_pose.p.data[0] = dir*(i*M_PI/(30*range)+0.1);
            end_effector_pose.p.data[1] = 0.325;
            end_effector_pose.p.data[2] = -0.05*sin(i*M_PI/range)+0.435;
            p_tracik_solver_->CartToJnt(p, end_effector_pose, result);
            p = result;
            vResult.push_back(result);
        }

//   // pub test
//    ros::Publisher pub=nh_.advertise<sensor_msgs::JointState>("joint_states",1);
//    ros::Rate rate(15);

//    while(ros::ok()){
//    KDL::JntArray jntArr;
//    for( i=0;i<vResult.size();++i)
//    {
//      sensor_msgs::JointState jointMsg;
//      jointMsg.header.stamp = ros::Time::now();

//      jntArr = vResult[i];
//      int n = jntArr.rows()*jntArr.columns();
//      for(int x=0;x<n;++x)
//      {
//        jointMsg.name.push_back(chain.getSegment(x).getJoint().getName());
//        jointMsg.position.push_back(jntArr(x));
//      }

//      pub.publish(jointMsg);
//      ros::spinOnce();
//      rate.sleep();
//    }
//    }
//    return;//test

        //trajectory goal
        control_msgs::FollowJointTrajectoryGoal g;
        //head
        //g.trajectory.header.frame_id = "base_link";
        g.trajectory.header.stamp = ros::Time::now();
        //names
        for(unsigned i=0;i<nbr;++i)
        {
            std::string name = chain.getSegment(i).getJoint().getName();
            g.trajectory.joint_names.push_back(name);
            ROS_INFO_STREAM("joint name : "<<name);
        }

        //loop result
        KDL::JntArray jntArr;
        double dur = 5.0;
        for(unsigned i=0;i<vResult.size();++i)
        {
            jntArr = vResult[i];
            
            //pt
        
            trajectory_msgs::JointTrajectoryPoint pt;

            int n=jntArr.rows()*jntArr.columns();
            for(int x=0;x<n;++x)
            {
                pt.positions.push_back(jntArr(x));
                pt.velocities.push_back(0);
            }

            pt.time_from_start = ros::Duration(dur);
            dur += 0.05;

            g.trajectory.points.push_back(pt);
            ROS_INFO_STREAM("  joint pos 1: "<<pt.positions[0]<<" joint pos 2: "<<pt.positions[1]<<" joint pos 3: "<<pt.positions[2]<<"  joint pos 4: "<<pt.positions[3]<<"  joint pos 5: "<<pt.positions[4]<<"  joint pos 6: "<<pt.positions[5]);
//      break;//test
            //servoj_moveto(jntArr,0.01,true);
            
             ROS_INFO_STREAM("vResult.size()="<<vResult.size());
             ROS_INFO_STREAM("i="<<i);
             ROS_INFO_STREAM("n="<<n);

        }
            //client_servoj_->sendGoal(g);
                client_servoj_->sendGoal(g);
                client_servoj_->waitForResult(ros::Duration());
                ROS_INFO_STREAM("  send order to robot ");

        

    if(client_servoj_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      client_servoj_->cancelGoal();
    }

    }


void MyRobot::servoj_sin_test(){
        std::vector<double> gravity_down_joints;
        nh_.getParam("gravity_down_joints", gravity_down_joints);
        KDL::JntArray gravity_down_jnt(6);
        for (int i = 0; i < 6; ++i) {
            gravity_down_jnt(i) = gravity_down_joints[i] * M_PI / 180;
        }
        ROS_INFO_STREAM("t0 ");
        servoj_moveto(gravity_down_jnt, 5, true);

        ROS_INFO_STREAM("t1 ");

        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(!bur_sub_);
        KDL::Frame original_P = frame_wrist3_base_;

        ros::Rate loop_rate(500);
        uint64_t nsec0 = ros::Time::now().toNSec();

        while(ros::ok()){
            ros::spinOnce();
            ROS_INFO_STREAM("z " << frame_wrist3_base_.p.z() );
            uint64_t time_now = ros::Time::now().toNSec() - nsec0;
            double time_now_d = time_now / 1e9;
             ROS_INFO_STREAM("time_now_d " << time_now_d );
            KDL::Frame p1 = original_P;
            KDL::JntArray q1;
            q1.resize(6);
            p1.p.data[2] = original_P.p.data[2]+ 0.1*sin(2*M_PI/5*time_now_d);
            p_tracik_solver_->CartToJnt(current_JntArr_,p1,q1);
            servoj_moveto(q1,0.01,true);
            loop_rate.sleep();
            if (time_now_d>20)
                break;
        }

    }

void MyRobot::draw_circle_servioj()
    {
        
        //圆心
        bool flag=true;
        double radius=0.2;//半径
        float i=0;
        const double error=1e-5;
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(!bur_sub_);
        
            KDL::Frame original_P = frame_wrist3_base_;
            KDL::Frame p1 = original_P;
            KDL::JntArray q1;
            q1.resize(6);
            p1.p.data[0] = original_P.p.data[0]+ radius;//x
            p1.p.data[1] = original_P.p.data[1];//y
            p_tracik_solver_->CartToJnt(current_JntArr_,p1,q1);
            servoj_moveto(q1,6,true);
            ROS_INFO_STREAM("x " << frame_wrist3_base_.p.x() );
            ROS_INFO_STREAM("y " << frame_wrist3_base_.p.y() );
            ROS_INFO_STREAM(" at the circle ");

            
       
        ros::Rate loop_rate(125);//2ms  125hz 0.008s 告诉机器人该做什么
        //到圆上
            
        uint64_t nsec0 = ros::Time::now().toNSec();
        ros::Rate rate( loop_rate );
        
        while(ros::ok())
        {
           
            ros::spinOnce();
        
            ROS_INFO_STREAM("x " << frame_wrist3_base_.p.x() );
            ROS_INFO_STREAM("y " << frame_wrist3_base_.p.y() );
            ROS_INFO_STREAM("current_JntSpeed" << current_JntSpeed(0));
            uint64_t time_now = ros::Time::now().toNSec() - nsec0;//时间清零
            double time_now_d = time_now /1e9;//ns
            ROS_INFO_STREAM("time_now_d " << time_now_d );
           // KDL::Frame p1 = original_P;
            KDL::JntArray q1;
            q1.resize(6);
            p1.p.data[0] = original_P.p.data[0]+ radius*cos(2*M_PI/10*time_now_d);//x
            p1.p.data[1] = original_P.p.data[1]+ radius*sin(2*M_PI/10*time_now_d);//y
            p_tracik_solver_->CartToJnt(current_JntArr_,p1,q1);
                 //loop_rate.sleep();
                 //sleep(5.0);
            servoj_moveto(q1,0.04,true);
           
            
            
            //record
            std::ofstream file3;
            std::ostringstream oss;
            int name_suffix = 1;
            oss << name_suffix;
            std::string filename = oss.str() + "_servoj_tracking_real_robot";
            std::string file = "/home/gl/catkin_ws/src/plan/record_data_circle" + filename + ".txt";
            file3.open(file.c_str(), std::ios::app);
            file3 << time_now_d << " ";
            file3 << p1.p.data[0]<< " " <<p1.p.data[1] << " " << p1.p.data[2]<< " "
                  << end_point_.data[0]<< " " << end_point_.data[1] << " "<<end_point_.data[2]<<" "
                  <<current_JntSpeed(0)<<" "<<current_JntSpeed(1)<<" "<<current_JntSpeed(2)<<" "<<current_JntSpeed(3)
                  <<" "<<current_JntSpeed(4)<<" "<<current_JntSpeed(5);
            file3 << std::endl;
            file3.close();
            loop_rate.sleep();
           // ros::shutdown();
             if (time_now_d>60)
                break;
        }
    }


    void MyRobot::force_interaction()
    {
        // repair wrench
        ROS_INFO_STREAM("begin to initialize wrench " );
        bsub_wrench_ = false;
        do{
            ros::spinOnce();
        }while(!bsub_wrench_);
        wrench_repair_ = wrench_base_;
//     ros::spin();
        // test force interaction controller
//        x_kd = 0.1;
//        x_ki = 0;
//        x_kd = 0;   // xyz PID parameters
//        r_kp = 0;
//        r_ki = 0;
//        r_kd = 0;   // rpy PID parameters
        //ForceInteraction controller1;
        //controller1.initialize();
        double force_base[6]={0,0,0,0,0,0};
        double delta_x[3] = {0,0,0};
        double delta_rpy[3] = {0,0,0};

        sub_ur_update();
        KDL::Frame target_frame = frame_wrist3_base_;//参考系
        KDL::Frame start_frame = frame_wrist3_base_;
        KDL::JntArray target_jnt(6);
        uint64_t sec0 = ros::Time::now().toSec();
        ROS_INFO_STREAM("start force  interaction " );
        ros::Rate loop_rate(500);

        while(ros::ok()){
            ros::spinOnce();
            // contact force protection
            double sum_force = wrench_base_.force.Norm();
            if (sum_force>50){
                ROS_ERROR("contact force is too big. stop moving");
                break;
            }
            for (int i = 0; i <3 ; ++i) {
                force_base[i]   = wrench_base_.force.data[i];   //六维力力矩传感器 force（前三个）
                force_base[i+3] = wrench_base_.torque.data[i];  //torque
            }

            target_frame.p = frame_wrist3_base_ .p;    // consider xyz without rpy rotation.
            double time_now_d = ros::Time::now().toSec() - sec0;
            //controller1.step(force_base,delta_x,delta_rpy);





            for (int j = 0; j < 3; ++j) 
            {
                if (abs(wrench_base_.force.data[j])<5) {
                    delta_x[j] = 0;
//                    controller1.initialize();
                }
                target_frame.p.data[j] +=delta_x[j]/1000.0;    // mm to m
            }
            ROS_INFO_STREAM("delta_xyz: "<< delta_x[0]<<" "<< delta_x[1]<<" "<< delta_x[2]<<" "
                                         << wrench_base_.force.data[0]<<" "<< wrench_base_.force.data[1]
                                         <<" "<<wrench_base_.force.data[2]<<" ");
           // move_line_dynamic(target_frame.p,0.1);
//            p_tracik_solver_->CartToJnt(current_JntArr_,target_frame,target_jnt);
//            servoj_moveto(target_jnt,0.002,false);
            loop_rate.sleep();
        }
    }
#endif