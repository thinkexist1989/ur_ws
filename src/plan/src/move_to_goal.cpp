#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
const double PI = M_PI;
//初始位姿
//double q_cur[6]={0.0,-PI,PI/2,-PI/2,PI/2,0.0};
std::vector<double> group_variable_values={0.00165,-1.57291,0.0427,-1.56864,0.00127,0.00317};
int main(int argc, char **argv)
{
    // Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "move_group_plan");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();
    
    // Get the arm planning group
    //moveit::planning_interface::MoveGroup plan_group("manipulator");
    moveit::planning_interface::MoveGroupInterface plan_group("manipulator");
    // Create a published for the arm plan visualization
    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    // Get the current RobotState, which will be used to set the arm
    // to one of the predefined group states, in this case home
    robot_state::RobotState current_state = *plan_group.getCurrentState();

    // We set the state values for this robot state to the predefined
    // group state values
    current_state.setToDefaultValues(current_state.getJointModelGroup("manipulator"), "group_variable_values");

    // We set the current state values to the target values
    plan_group.setJointValueTarget(current_state);


    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan;
    if (plan_group.plan(goal_plan))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan.start_state_;
        display_msg.trajectory.push_back(goal_plan.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);
        
        plan_group.move();
        plan_group.execute(goal_plan);
    }

    ros::shutdown();

    return 0;
}