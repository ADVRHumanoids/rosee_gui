/*
 * Copyright 2020 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SINGLEACTIONTIMEDGROUPBOX_H
#define SINGLEACTIONTIMEDGROUPBOX_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <iostream>

#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>

#include <rosee_gui/ActionTimedElement.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosee_msg/action/rosee_command.hpp>
#include <rosee_msg/msg/grasping_action.hpp> //msg
#include "Action.h"
#include "ActionPrimitive.h"

/**
 * @todo write docs
 */
class SingleActionTimedGroupBox : public QGroupBox {
    
    Q_OBJECT
public :
    //using GraspingActionROS = rosee_msg::action::ROSEECommand;
    using GoalHandleGraspingActionROS = rclcpp_action::ClientGoalHandle<rosee_msg::action::ROSEECommand>;
    
    explicit SingleActionTimedGroupBox (const rclcpp::Node::SharedPtr node, const rosee_msg::msg::GraspingAction graspingAction,
                                QWidget* parent = 0);
    
    rclcpp_action::Client<rosee_msg::action::ROSEECommand>::SharedPtr action_client;
    void goal_response_callback(std::shared_future<GoalHandleGraspingActionROS::SharedPtr> future);

    void feedback_callback(GoalHandleGraspingActionROS::SharedPtr, 
                          const std::shared_ptr<const rosee_msg::action::ROSEECommand::Feedback> feedback);
    void result_callback(const GoalHandleGraspingActionROS::WrappedResult & result);

    void resetAll();

private: 
    rclcpp::Node::SharedPtr _node;
    
    QGridLayout *grid;
    QLabel* windowLabel;
    QPushButton *send_button;
    
    unsigned int rosMsgSeq;
    std::string actionName;
    
    virtual void sendActionRos();
    void setRosActionClient ();

private slots:
    void sendBtnClicked();
    
};

#endif // SINGLEACTIONTIMEDGROUPBOX_H
