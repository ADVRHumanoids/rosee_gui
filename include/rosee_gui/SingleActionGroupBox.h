#ifndef SINGLEACTIONGROUPBOX_H
#define SINGLEACTIONGROUPBOX_H

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
#include <rosee_msg/GraspingAction.h> //msg
#include <end_effector/GraspingActions/Action.h>
#include <end_effector/GraspingActions/ActionPrimitive.h>

#include <actionlib/client/simple_action_client.h>

/**
 * TODO better to pass a pub pointer to have only a single publisher for all gui?
 */
class SingleActionGroupBox: public QGroupBox
{
    Q_OBJECT
public:
    explicit SingleActionGroupBox(ros::NodeHandle *nh, std::string actionName, 
                                           ROSEE::Action::Type actionType, QWidget* parent=0);
    
    virtual void resetAll();
        
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

#endif // SINGLEACTIONGROUPBOX_H
