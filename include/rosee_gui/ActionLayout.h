#ifndef ACTIONLAYOUT_H
#define ACTIONLAYOUT_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QCheckBox>
#include <QGroupBox>
#include <QProgressBar>

#include <iostream>
#include <memory.h>

#include <ros/ros.h>
#include <ros_end_effector/EEGraspControl.h>
#include <rosee_msg/ROSEECommandAction.h>
#include <actionlib/client/simple_action_client.h>

//TODO or 0-box 1-box ?? if we have to send only a percentage and a certain number of string...
enum MsgType {  GENERIC, TRIG, PINCH };

//TODO: now graspmsg is used for GENERIC, but a better name should be generic msg (one where we only give the percentage)
class ActionLayout: public QGroupBox
{
    Q_OBJECT
public:
    explicit ActionLayout(std::string actionName, QWidget* parent=0);
    
    virtual void setRosPub (ros::NodeHandle * nh, std::string topicName, MsgType msgType = GENERIC);
    
    virtual void ActionLayout::setRosActionClient ( ros::NodeHandle * nh, std::string rosActionName);



protected:
    QGridLayout *grid;
    unsigned int rosMsgSeq;

    QPushButton *send_button; //protected so derived class can enable/disable if necessary
    ros::Publisher actionPub;
    std::shared <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction>> action_client;
    
    /**
     * @brief msgType for this class is always GENERIC, derived class will modified it in the setRosPub overriden function
     */
    MsgType msgType;

    /**
     * @brief getSpinBoxPercentage getter for derived class
     * @return (0.0 - 1.0 double) the percentage displayed in the spinBox. The spinBox and slider have a syncro-same value
     */
    double getSpinBoxPercentage();

private:
    QSlider *slider_percentage;
    QSpinBox *spinBox_percentage;
    QProgressBar *progressBar;

    virtual void sendActionRos();

signals:

private slots:
    void slotSliderReceive(int value);
    void sendBtnClicked();


};

#endif // ACTIONLAYOUT_H
