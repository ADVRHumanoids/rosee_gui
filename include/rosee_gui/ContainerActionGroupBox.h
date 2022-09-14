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

#include <rosee_msg/msg/grasping_action.hpp>
#include <rosee_msg/srv/grasping_actions_available.hpp>
#include <rosee_msg/msg/grasping_primitive_aggregated.hpp>
#include <rosee_msg/srv/grasping_primitive_aggregated_available.hpp>
#include <rosee_msg/srv/selectable_pair_info.hpp>
#include "Action.h"

/**
 * @todo write docs
 */
class ContainerActionGroupBox : public QGroupBox
{

    Q_OBJECT

public:
    
    explicit ContainerActionGroupBox (const rclcpp::Node::SharedPtr node, QWidget* parent = 0);
    
private:
    QGridLayout *grid;
    rclcpp::Node::SharedPtr _node;
    std::vector <rosee_msg::msg::GraspingPrimitiveAggregated> primitivesAggregatedAvailableMsg;
    std::vector <rosee_msg::msg::GraspingAction> genericsAvailableMsg;
    std::vector <rosee_msg::msg::GraspingAction> timedsAvailableMsg;
    QPushButton *resetButton;

    
    std::map < std::string, std::vector<std::string> > getPairMap(
        std::string action_name, std::vector<std::string> elements);
    
    void getInfoServices() ;

private slots:
    void resetButtonClicked();
    
};

#endif // CONTAINERACTIONGROUPBOX_H
