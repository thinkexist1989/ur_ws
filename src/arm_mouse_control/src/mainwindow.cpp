#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    spinner(1),
    move_group(PLANNING_GROUP)
{
    ui->setupUi(this);
    spinner.start();

    joint_model_group_ptr = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("arm_control_gui", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO_NAMED("arm_control_gui", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    geometry_msgs::Pose start_pose;
    start_pose.orientation.w = 1.0;
    start_pose.position.x = 0.7;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.7;
    move_group.setPoseTarget(start_pose);

    move_group.move();

}

MainWindow::~MainWindow()
{
    delete ui;
    ros::shutdown();
}


void MainWindow::on_buttonUp_clicked()
{
    std::cout << "Up button clicked!" << std::endl;
    cartesian_move(0, 0, 0.1);
}

void MainWindow::on_buttonDown_clicked()
{
    std::cout << "Down button clicked!" << std::endl;
    cartesian_move(0, 0, -0.1);
}

void MainWindow::on_buttonLeft_clicked()
{
    std::cout << "Left button clicked!" << std::endl;
    cartesian_move(0, 0.1, 0);
}

void MainWindow::on_buttonRight_clicked()
{
    std::cout << "Right button clicked!" << std::endl;
    cartesian_move(0, -0.1, 0);
}

void MainWindow::cartesian_move(double x_step, double y_step, double z_step)
{
    geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
    pose.position.x += x_step;
    pose.position.y += y_step;
    pose.position.z += z_step;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, EEF_STEP, JUMP_THESHOLD, trajectory);
    ROS_INFO_NAMED("arm_control_gui", "Cartesian Path %.2f%% is achieved.", fraction*100);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group.execute(plan);
}