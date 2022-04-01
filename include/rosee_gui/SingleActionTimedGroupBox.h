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

#include <iostream>

#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <ros/ros.h>
#include <rosee_gui/ActionTimedElement.h>

#include <end_effector/GraspingActions/Action.h> //for action types
#include <end_effector/GraspingActions/ActionPrimitive.h> //for action types
#include <rosee_msg/ROSEECommandAction.h>
#include <rosee_msg/GraspingAction.h> //msg

#include <actionlib/client/simple_action_client.h>

/**
 * @todo write docs
 */
class SingleActionTimedGroupBox : public QGroupBox {
    
    Q_OBJECT
public :
    explicit SingleActionTimedGroupBox (ros::NodeHandle* nh, rosee_msg::GraspingAction graspingAction,
                                QWidget* parent = 0);
    
    std::shared_ptr <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction> > action_client;
    void doneCallback(const actionlib::SimpleClientGoalState& state,
            const rosee_msg::ROSEECommandResultConstPtr& result);
    void activeCallback();
    void feedbackCallback(const rosee_msg::ROSEECommandFeedbackConstPtr& feedback);
    
    void resetAll();

private: 
    QGridLayout *grid;
    QLabel* windowLabel;
    QPushButton *send_button;
    
    unsigned int rosMsgSeq;
    std::string actionName;
    
    virtual void sendActionRos();
    void setRosActionClient ( ros::NodeHandle * nh);

private slots:
    void sendBtnClicked();
    
};

#endif // SINGLEACTIONTIMEDGROUPBOX_H
