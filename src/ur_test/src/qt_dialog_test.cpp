#include <QApplication>
#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QObject>

#include <iostream>

int main(int argc, char** argv)
{
    QApplication a(argc, argv);
    QDialog w;
    w.resize(400,500);
    QLabel label(&w);
    label.move(120, 120);
    label.setText(QObject::tr("Hello World! I'm Qt"));
    
    QPushButton button(&w);
    button.move(300, 120);
    button.setText(QObject::tr("Click Me!"));
    QObject::connect(&button, &QPushButton::clicked, [=](){
        std::cout << "button clicked"<< std::endl;
    });
    w.show();

    return a.exec();
}