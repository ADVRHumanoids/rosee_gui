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

MainWindow::MainWindow(ros::NodeHandle *nh, QWidget *parent) : QWidget(parent) {

    tabWidget = new QTabWidget;
    
    tabWidget->addTab(new Window(nh, parent), tr("General"));
    tabWidget->addTab(new Window(nh, parent), tr("tab2"));
    
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(tabWidget);
    setLayout(layout);

  

}


