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

#include <rosee_gui/JointStateContainer.h>

JointStateContainer::JointStateContainer (ros::NodeHandle* nh, QWidget *parent) : QVBoxLayout(parent) {
 
    jointStateTable = new JointStateTable(nh, 0,0, parent);
    QGridLayout* jointStateTableOptions = new QGridLayout;
    
    QCheckBox* posBox = new QCheckBox ( "position");
    posBox->setToolTip("Check to show position of joints");
    QCheckBox* velBox = new QCheckBox ( "velocity");
    posBox->setToolTip("Check to show velocity of joints");
    QCheckBox* effBox = new QCheckBox ( "effort");
    posBox->setToolTip("Check to show effort of joints");
    QCheckBox* actBox = new QCheckBox ( "not actuated joints");
    posBox->setToolTip("Check to show also the state of not actuated joints");
    
    jointStateTableOptions->addWidget(posBox, 0, 0);
    jointStateTableOptions->addWidget(velBox, 0, 1);
    jointStateTableOptions->addWidget(effBox, 0, 2);
    jointStateTableOptions->addWidget(actBox, 1, 0, 1, 3); //spawn so it is in the middle

    this->addWidget(jointStateTable);
    this->addLayout(jointStateTableOptions);
    
    
}

