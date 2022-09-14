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

#ifndef JOINTSTATETABLE_H
#define JOINTSTATETABLE_H

#include <QWidget>
#include <QTableWidget>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rosee_gui/RobotDescriptionHandler.h>

using std::placeholders::_1;

/**
 * @todo write docs
 */
class JointStateTable : public QTableWidget
{
    Q_OBJECT
    
public:
    explicit JointStateTable(rclcpp::Node::SharedPtr node, 
                             std::vector<std::string> jointNames,
                             QWidget *parent = nullptr);
    
private:
    bool setJointStateSub(rclcpp::Node::SharedPtr node);
    void jointStateClbk ( const sensor_msgs::msg::JointState::SharedPtr msg );
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointPosSub;
    sensor_msgs::msg::JointState jointStateMsg;
    
    std::map <std::string, int> jointNameRowMap;
    
};

#endif // JOINTSTATETABLE_H
