#ifndef SINGLEACTIONGROUPBOX_H
#define SINGLEACTIONGROUPBOX_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosee_msg/action/rosee_command.hpp>
#include <rosee_msg/msg/grasping_action.hpp> //msg
#include "Action.h"
#include "ActionPrimitive.h"

/**
 * TODO better to pass a pub pointer to have only a single publisher for all gui?
 */
class SingleActionGroupBox: public QGroupBox
{
    Q_OBJECT
public:
    //using GraspingActionROS = rosee_msg::action::ROSEECommand;
    using GoalHandleGraspingActionROS = rclcpp_action::ClientGoalHandle<rosee_msg::action::ROSEECommand>;
    
    explicit SingleActionGroupBox(const rclcpp::Node::SharedPtr node, std::string actionName, 
                                           ROSEE::Action::Type actionType, QWidget* parent=0);
    
    virtual void resetAll();
        
protected:
    QGridLayout *grid;
    unsigned int rosMsgSeq;
    std::string actionName;
    ROSEE::Action::Type actionType;

    QPushButton *send_button; //protected so derived class can enable/disable if necessary
    rclcpp_action::Client<rosee_msg::action::ROSEECommand>::SharedPtr action_client;

    void goal_response_callback(std::shared_future<GoalHandleGraspingActionROS::SharedPtr> future);
    void feedback_callback(GoalHandleGraspingActionROS::SharedPtr, 
                          const std::shared_ptr<const rosee_msg::action::ROSEECommand::Feedback> feedback);
    void result_callback(const GoalHandleGraspingActionROS::WrappedResult & result);
    
    /**
     * @brief getSpinBoxPercentage getter for derived class
     * @return (0.0 - 1.0 double) the percentage displayed in the spinBox. The spinBox and slider have a syncro-same value
     */
    double getSpinBoxPercentage();
    

private:
    
    rclcpp::Node::SharedPtr _node;
    QSlider *slider_percentage;
    QSpinBox *spinBox_percentage;
    QProgressBar *progressBar;

    virtual void sendActionRos();
    void setRosActionClient ();
    

signals:

private slots:
    void slotSliderReceive(int value);
    void sendBtnClicked();

};

#endif // SINGLEACTIONGROUPBOX_H
