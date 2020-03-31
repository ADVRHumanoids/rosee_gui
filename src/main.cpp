#include <QApplication>
#include <QProgressBar>
#include <QSlider>
#include <QPushButton>

#include <rosee_gui/Window.h>
#include <rosee_gui/TimerHandler.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    QApplication app (argc, argv);

    ros::init (argc, argv, "rosee_GUI");
    ros::NodeHandle nh;
    
    TimerHandler tHandler(10);

    Window window(&nh);

    window.show();
    
    int appReturn = app.exec();
    //app.exec is blocking
    
    ros::shutdown();

    return appReturn;
}



