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

#include <rosee_gui/MainWindow.h>

MainWindow::MainWindow(ros::NodeHandle *nh, QWidget *parent) : QTabWidget(parent) {
    
    std::string urdf_file, srdf_file;
    
    nh->getParam("robot_description", urdf_file);
    nh->getParam("robot_description_semantic", srdf_file);

    robotDescriptionHandler = std::make_shared<RobotDescriptionHandler>(urdf_file, srdf_file);

    addTab(new TabAction(nh, robotDescriptionHandler,  parent), tr("Action"));
    
    addTab(new JointMonitorWidget (nh, robotDescriptionHandler,  parent), tr("RobotState"));

}


