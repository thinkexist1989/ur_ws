#ifndef ROBOTMOTION_H
#define ROBOTMOTION_H

#include <ros/ros.h>           //ros相关头文件
#include <trac_ik/trac_ik.hpp> //trac_ik头文件
#include <eigen3/Eigen/Eigen>  // Eigen3头文件

class RobotMotion
{
public:
    RobotMotion() = default; // 默认构造函数 c++11特性
};

#endif