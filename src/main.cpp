#include <QApplication>
#include <QProgressBar>
#include <QSlider>
#include <QPushButton>

#include <rosee_gui/MainWindow.h>
#include <rosee_gui/TimerHandler.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{

    QApplication app (argc, argv);

    
    rclcpp::init ( argc, argv );
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("rosee_GUI");
    
    TimerHandler tHandler(node, 100);
    
    MainWindow mainwindow(node);

    mainwindow.show();
    
    int appReturn = app.exec();
    //app.exec is blocking
    
    rclcpp::shutdown();

    return appReturn;
}



