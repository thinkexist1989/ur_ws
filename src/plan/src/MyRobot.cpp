
#include "MyRobot.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "catch_control");
    ros::NodeHandle nh;
    std::string chain_start, chain_end, urdf_param;
    double timeout;
    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));
    std::string UR_name_prefix;
    nh.getParam("UR_name_prefix", UR_name_prefix);
    ROS_INFO_STREAM("UR_name_prefix: " << UR_name_prefix);

    MyRobot UR5e(nh, urdf_param, "base_link", "tool0", timeout, UR_name_prefix);
    ros::Duration(0.5).sleep();
    // ros::spinOnce();
    getchar();

    ros::AsyncSpinner spin(1);
    spin.start();
    //int loop_rate = 30;
    //ros::Rate rate( loop_rate );

    //double joint_start_arr[]={-0.3015,-2.04182,-1.863,-2.409,4.719,0.003};
    double joint_start_arr[] = {-0.29629, -2.04182, -1.863, -0.855, 1.53311, 0.090};
    KDL::JntArray joint_start(6);

    for (int i = 0; i < 6; i++)
    {
        joint_start(i) = joint_start_arr[i];
    }

    //UR5e.move_joints(joint_start,1.0);
    UR5e.servoj_moveto(joint_start, 6, true);
    ROS_INFO_STREAM("star");

    getchar();

    std::cout << "Continue...." << std::endl;

    // std::vector<double> joint={-0.3015,-2.04182,-1.863,-2.409,4.719,0.003};
    // UR5e.move_joint_to_star(joint);//运动到初始状态
    //待改进
    //UR5e.draw_circle();//draw circle input center_x,center_y,force need to do in the next day
    // UR5e.servoj_sin_test();

    //UR5e.draw_circle_servioj();

    // UR5e.draw_sin();
    UR5e.draw_circle_servioj();
    // UR5e.force_interaction();

    ros::shutdown();

    return 0;
}
