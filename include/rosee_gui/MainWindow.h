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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <rclcpp/rclcpp.hpp>
#include <QtGlobal> //for QT_VERSION flag
#include <QWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <rosee_gui/TabAction.h>

#include <rosee_gui/RobotDescriptionHandler.h>

#if SECOND_TAB_CODE
    //TODO solve this relative include
    #include "../../src/joint_state_gui/joint_monitor_widget.h"
#endif

/**
 * @todo write docs
 * 
 * We declare directly the main window as the qtab widget. 
 * The alternative would be declare it as QWidget and then use a QTabWidget
 * as a member, then creating a layout... Not necessary now, because we only have one
 * son. In future, if we want to add something external to the tab widget, we will
 * use this approach (that is more similar to the qdialog qt tutorial)
 */
class MainWindow : public  QTabWidget
{
    Q_OBJECT
    
public:
    explicit MainWindow(const rclcpp::Node::SharedPtr node, QWidget *parent = 0);

private:
    std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler;

};


#endif // MAINWINDOW_H

