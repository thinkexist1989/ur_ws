#include <QApplication>
#include "mainwindow.h"

#include <ros/ros.h>

#include <QDebug>
#include <QDir>
#include <QStyleFactory>

int main(int argc, char** argv)
{  
    // QApplication::setStyle(QStyleFactory::create("Fusion"));
    ros::init(argc, argv, "arm_control_gui");
    QApplication a(argc, argv);
    MainWindow w;
    w.show(); 

    return a.exec();
}