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

MainWindow::MainWindow(const rclcpp::Node::SharedPtr node, QWidget *parent) : QTabWidget(parent) {
    
    std::string urdf_file, srdf_file;
    
    node->declare_parameter("urdf_path", "");
    node->declare_parameter("srdf_path", "");
    
    node->get_parameter("urdf_path", urdf_file);
    node->get_parameter("srdf_path", srdf_file);

    robotDescriptionHandler = std::make_shared<RobotDescriptionHandler>(node, urdf_file, srdf_file);

    addTab(new TabAction(node, robotDescriptionHandler,  parent), tr("Action"));
    
#if SECOND_TAB_CODE
    addTab(new JointMonitorWidget (node, robotDescriptionHandler,  parent), tr("RobotState"));
#endif

}


