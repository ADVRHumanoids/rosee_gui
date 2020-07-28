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

#include <rosee_msg/ROSEECommandAction.h>
#include <rosee_msg/ActionInfo.h> //msg
#include <ros_end_effector/Action.h>
#include <ros_end_effector/ActionPrimitive.h>

#include <actionlib/client/simple_action_client.h>

/**
 * TODO better to pass a pub pointer to have only a single publisher for all gui?
 */
class ActionLayout: public QGroupBox
{
    Q_OBJECT
public:
    explicit ActionLayout(ros::NodeHandle *nh, rosee_msg::ActionInfo, QWidget* parent=0);
        
protected:
    QGridLayout *grid;
    unsigned int rosMsgSeq;
    std::string actionName;
    ROSEE::Action::Type actionType;

    QPushButton *send_button; //protected so derived class can enable/disable if necessary
    std::shared_ptr <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction> > action_client;
    void doneCallback(const actionlib::SimpleClientGoalState& state,
            const rosee_msg::ROSEECommandResultConstPtr& result);
    void activeCallback();
    void feedbackCallback(const rosee_msg::ROSEECommandFeedbackConstPtr& feedback);

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
    void setRosActionClient ( ros::NodeHandle * nh);
    



signals:

private slots:
    void slotSliderReceive(int value);
    void sendBtnClicked();


};

#endif // ACTIONLAYOUT_H
