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

#ifndef TABSENSORSSTATE_H
#define TABSENSORSSTATE_H

#include <QWidget>
#include <QTableWidget>
#include <ros/ros.h>

#include <ros_msg_parser/ros_parser.hpp>


/**
 * @todo write docs
 */
class TabSensorsState : public QWidget
{
    Q_OBJECT
public:
    
    explicit TabSensorsState ( ros::NodeHandle* nh, std::vector<std::string> topicNames,
                               QWidget* parent = 0);
    
private: 
    std::map<std::string, QTableWidget*> _tables ;
    std::vector<ros::Subscriber> _subscribers;
    
    void topicCallback(const RosMsgParser::ShapeShifter& msg,
                   const std::string &topic_name,
                   RosMsgParser::ParsersCollection& rosIntroParsers);


};

#endif // TABSENSORSSTATE_H
