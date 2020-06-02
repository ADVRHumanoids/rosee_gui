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

#ifndef SENSORSTATETABLE_H
#define SENSORSTATETABLE_H

#include <QTableWidget>
#include <QWidget>
#include <QHeaderView>

#include <ros/ros.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h> //necessary for introspection

struct SensorsStateOption {
    std::string topicName;
    std::vector <std::string> columnNames;
    std::string rowLabel;
};

/**
 * @todo write docs
 */
class SensorStateTable :  public QTableWidget
{
public:

    explicit SensorStateTable(ros::NodeHandle* nh, SensorsStateOption opt, QWidget* parent = 0);
    
private:
    bool initialized;
    ros::Subscriber _subscriber;
    RosIntrospection::Parser rosIntroParser;
    SensorsStateOption opt;
    
    void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg);


};

#endif // SENSORSTATETABLE_H
