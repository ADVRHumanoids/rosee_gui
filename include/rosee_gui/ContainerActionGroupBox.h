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

#ifndef CONTAINERACTIONGROUPBOX_H
#define CONTAINERACTIONGROUPBOX_H

#include <QGridLayout>
#include <QGroupBox>
#include <QWidget>
#include <memory.h>

#include <rosee_gui/SingleActionBoxesGroupBox.h>
#include <rosee_gui/SingleActionGroupBox.h>
#include <rosee_gui/SingleActionTimedGroupBox.h>

#include <rosee_msg/GraspingAction.h>
#include <rosee_msg/GraspingActionsAvailable.h>
#include <rosee_msg/GraspingPrimitiveAggregated.h>
#include <rosee_msg/GraspingPrimitiveAggregatedAvailable.h>
#include <rosee_msg/SelectablePairInfo.h>
#include <end_effector/GraspingActions/Action.h>

/**
 * @todo write docs
 */
class ContainerActionGroupBox : public QGroupBox
{

    Q_OBJECT

public:
    
    explicit ContainerActionGroupBox (ros::NodeHandle* nh, QWidget* parent = 0);
    
private:
    QGridLayout *grid;
    ros::NodeHandle* nh;
    std::vector <rosee_msg::GraspingPrimitiveAggregated> primitivesAggregatedAvailableMsg;
    std::vector <rosee_msg::GraspingAction> genericsAvailableMsg;
    std::vector <rosee_msg::GraspingAction> timedsAvailableMsg;
    QPushButton *resetButton;

    
    std::map < std::string, std::vector<std::string> > getPairMap(
        std::string action_name, std::vector<std::string> elements);
    
    void getInfoServices() ;

private slots:
    void resetButtonClicked();
    
};

#endif // CONTAINERACTIONGROUPBOX_H
