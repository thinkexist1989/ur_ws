#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>

#include <RobotMotion.hpp>

#define PLANNING_GROUP "manipulator"
#define JUMP_THESHOLD 0.0
#define EEF_STEP 0.01

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    ros::NodeHandle n;
    ros::AsyncSpinner spinner;

    RobotMotion *ur5e;

    moveit::planning_interface::MoveGroupInterface move_group;
    const moveit::core::JointModelGroup *joint_model_group_ptr;

    void cartesian_move(double x_step, double y_step, double z_step);

private:
    Ui::MainWindow *ui;

public slots:
    void on_buttonUp_clicked();
    void on_buttonDown_clicked();
    void on_buttonLeft_clicked();
    void on_buttonRight_clicked();
    void on_buttonInit_clicked();
};

#endif // MAINWINDOW_H
