#include <QApplication>
#include <QProgressBar>
#include <QSlider>
#include <QPushButton>

#include <rosee_gui/Window.h>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    QApplication app (argc, argv);

    ros::init (argc, argv, "rosee_GUI");
    ros::NodeHandle nh;

    Window window(&nh);

    window.show();

    int appReturn = app.exec();

    // app.exec is blocking, if we are here user had closed the gui
    ros::shutdown();

    return appReturn;
}


