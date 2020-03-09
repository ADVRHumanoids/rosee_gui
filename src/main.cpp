#include <QApplication>
#include <QProgressBar>
#include <QSlider>
#include <QPushButton>

#include <rosee_gui/Window.h>
#include <ros/ros.h>

#include <thread>

void rosSpin_func() {

    ros::spin();
}


int main(int argc, char **argv)
{
    QApplication app (argc, argv);

    ros::init (argc, argv, "rosee_GUI");
    ros::NodeHandle nh;

    Window window(&nh);

    window.show();
    
    //TODO IS IT SAFE?? IS IT CORRECT?
    //BUG with this, the gui crash when we send an action and the previous one
    // was not completed
    std::thread rosSpinner(rosSpin_func);
    int appReturn = app.exec();

    //app.exec is blocking
    rosSpinner.join();
    ros::shutdown();

    return appReturn;
}



