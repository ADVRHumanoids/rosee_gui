#include <QApplication>
#include <QProgressBar>
#include <QSlider>
#include <QPushButton>

#include <rosee_gui/MainWindow.h>
#include <rosee_gui/TimerHandler.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    QApplication app (argc, argv);

    ros::init (argc, argv, "rosee_GUI");
    ros::NodeHandle nh;
    
    TimerHandler tHandler(100);
    
    
    MainWindow mainwindow(&nh);

    mainwindow.show();
    
    int appReturn = app.exec();
    //app.exec is blocking
    
    ros::shutdown();

    return appReturn;
}



