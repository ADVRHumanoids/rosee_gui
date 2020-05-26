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

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <rosee_gui/RobotDescriptionHandler.h>

/**
 * @todo write docs
 */
class JointStateTable : public QTableWidget
{
    Q_OBJECT
    
public:
    explicit JointStateTable(ros::NodeHandle* nh, 
                             std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler, 
                             QWidget *parent = nullptr);
    
private:
    bool setJointStateSub(ros::NodeHandle* nh);
    void jointStateClbk ( const sensor_msgs::JointStateConstPtr& msg );
    
    ros::Subscriber jointPosSub;
    sensor_msgs::JointState jointStateMsg;
    
    std::map <std::string, int> jointNameRowMap;
    
};

#endif // JOINTSTATETABLE_H
