#ifndef JOINT_MONITOR_WIDGET_H
#define JOINT_MONITOR_WIDGET_H

#include <QWidget>
#include <QTimer>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>
#include <urdf_parser/urdf_parser.h>

//TODO solve this
#include "../../include/rosee_gui/RobotDescriptionHandler.h"

#include "bar_plot_widget.h"
#include "joint_state_widget.h"
#include "../chart/chart.h"


class JointMonitorWidget : public QWidget
{

public:

    explicit JointMonitorWidget(ros::NodeHandle* nh, std::shared_ptr<RobotDescriptionHandler>, QWidget *parent = nullptr);


    BarPlotWidget * barplot_wid;
    JointStateWidget * jstate_wid;

private:

    ChartWidget * _chart;
    QTimer * _timer;
    ros::Subscriber _jstate_sub;
    bool _valid_msg_recv;
    bool _widget_started;
    std::vector<std::string> _jnames;
    urdf::ModelInterfaceSharedPtr _urdf;
    
    std::shared_ptr<RobotDescriptionHandler> _robotDescriptionHandler;

    void on_timer_event();
    void on_jstate_recv(const sensor_msgs::JointStateConstPtr& msg);

    std::map<std::string, int> _jidmap;

};

#endif // JOINT_MONITOR_WIDGET_H
