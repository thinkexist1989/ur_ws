#include <QApplication>
#include "mainwindow.h"

#include <QDebug>
#include <QDir>
#include <QStyleFactory>

int main(int argc, char** argv)
{  
    // QApplication::setStyle(QStyleFactory::create("Fusion"));
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}