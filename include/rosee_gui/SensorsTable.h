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

#ifndef SENSORSTABLE_H
#define SENSORSTABLE_H

#include <QTableWidget>
#include <ros/ros.h>

/**
 * @todo write docs
 */
class SensorsTable : public QTableWidget
{
    Q_OBJECT
public:
    /**
     * Constructor
     *
     * @param parent TODO
     */
    explicit SensorsTable ( ros::NodeHandle* nh, std::string topicName, QWidget* parent = 0 );

};

#endif // SENSORSTABLE_H
