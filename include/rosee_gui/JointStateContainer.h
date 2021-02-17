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

#ifndef JOINTSTATECONTAINER_H
#define JOINTSTATECONTAINER_H

#include <QWidget>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>

#include <rosee_gui/JointStateTable.h>
#include <rosee_gui/RobotDescriptionHandler.h>

#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>

#include <ros/ros.h>

/**
 * @todo write docs
 */
class JointStateContainer : public QVBoxLayout
{
    Q_OBJECT
    
public:
    explicit JointStateContainer(ros::NodeHandle* nh,  std::shared_ptr<RobotDescriptionHandler>, QWidget *parent = nullptr);
    
public slots:
    void showPositionCol (int);
    void showVelocityCol (int);
    void showEffortCol (int);
    void showActuatedJoints(int);
    void showMimicJoints(int);
    void showPassiveJoints (int);
    
private: 
    JointStateTable* activeJointStateTable;
    JointStateTable* mimicJointStateTable;
    JointStateTable* passiveJointStateTable;
    
    QPushButton *storePosButton;
    std::string storePosFilename;
    XBot::MatLogger2::Ptr matLogger;

    
    QLabel* activeJointslabel;
    QLabel* mimicJointslabel;
    QLabel* passiveJointslabel;
    
private slots:
    void storePosButtonClick();
    void initStoreButton();

};

#endif // JOINTSTATECONTAINER_H
