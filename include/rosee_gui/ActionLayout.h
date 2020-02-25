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

#include <iostream>

#include <ros/ros.h>
#include <ros_end_effector/EEGraspControl.h>

//TODO or 0-box 1-box ?? if we have to send only a percentage and a certain number of string...
enum MsgType {  GENERIC, TRIG, PINCH };

//TODO: now graspmsg is used, but a better name should be generic msg (one where we only give the percentage)
class ActionLayout: public QGroupBox
{
    Q_OBJECT
public:
    explicit ActionLayout(std::string actionName, QWidget* parent=0);
    
    virtual void setRosPub (ros::NodeHandle * nh, std::string topicName, MsgType msgType = GENERIC);


protected:
    QGridLayout *grid;
    unsigned int rosMsgSeq;

    QPushButton *send_button; //protected so derived class can enable/disable if necessary
    ros::Publisher actionPub;
    /**
     * @brief msgType for this class is always GENERIC, derived class will modified it in the setRosPub overriden function
     */
    MsgType msgType;

    /**
     * @brief getSpinBoxPercentage getter for derived class
     * @return the percentage displayed in the spinBox. The spinBox and slider have a syncro-same value
     */
    int getSpinBoxPercentage();

private:
    QLabel* windowLabel;
    QSlider *slider_percentage;
    QSpinBox *spinBox_percentage;

    virtual void sendActionRos();




signals:

private slots:
    void slotSliderReceive(int value);
    void sendBtnClicked();


};

#endif // ACTIONLAYOUT_H
