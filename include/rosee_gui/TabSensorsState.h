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
#include <QGridLayout>
#include <rosee_gui/SensorStateTable.h>
#include <ros/ros.h>


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
    std::map<std::string, SensorStateTable*> _tables ;

};

#endif // TABSENSORSSTATE_H
