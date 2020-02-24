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

    app.exec();

    std::cout << "Qt gui closed, press ctrl+C to end program..." << std::endl;
    while(ros::ok()) {}

    return 0;
}
