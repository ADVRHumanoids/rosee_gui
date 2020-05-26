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

JointStateContainer::JointStateContainer (ros::NodeHandle* nh,
                                          std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler,
                                          QWidget *parent) : QVBoxLayout(parent) {
 
    jointStateTable = new JointStateTable(nh, 0,0, parent);
    QGridLayout* jointStateTableOptions = new QGridLayout;
    
    QCheckBox* posBox = new QCheckBox ( "position");
    posBox->setToolTip("Check to show position of joints");
    posBox->setChecked(true);
    QObject::connect( posBox, SIGNAL (stateChanged(int)), 
                      this,   SLOT(showPositionCol(int)) );
    
    QCheckBox* velBox = new QCheckBox ( "velocity");
    velBox->setToolTip("Check to show velocity of joints");
    velBox->setChecked(true);
    QObject::connect( velBox, SIGNAL (stateChanged(int)), 
                      this,   SLOT(showVelocityCol(int)) );
    
    QCheckBox* effBox = new QCheckBox ( "effort");
    effBox->setToolTip("Check to show effort of joints");
    effBox->setChecked(true);
    QObject::connect( effBox, SIGNAL (stateChanged(int)), 
                      this,   SLOT(showEffortCol(int)) );
    
    QCheckBox* nonActBox = new QCheckBox ( "not actuated joints");
    nonActBox->setToolTip("Check to show also the state of not actuated joints");
    nonActBox->setChecked(true);
    
    jointStateTableOptions->addWidget(posBox, 0, 0);
    jointStateTableOptions->addWidget(velBox, 0, 1);
    jointStateTableOptions->addWidget(effBox, 0, 2);
    jointStateTableOptions->addWidget(nonActBox, 1, 0, 1, 3); //spawn so it is in the middle

    this->addWidget(jointStateTable);
    this->addLayout(jointStateTableOptions);
    
}

void JointStateContainer::showPositionCol(int state) {
    
    if (state == Qt::Checked) {
        jointStateTable->setColumnHidden(0, false);
        
    } else {
        jointStateTable->setColumnHidden(0, true);
    }
}

void JointStateContainer::showVelocityCol(int state) {
    
    if (state == Qt::Checked) {
        jointStateTable->setColumnHidden(1, false);
        
    } else {
        jointStateTable->setColumnHidden(1, true);
    }
}

void JointStateContainer::showEffortCol(int state) {
    
    if (state == Qt::Checked) {
        jointStateTable->setColumnHidden(2, false);
        
    } else {
        jointStateTable->setColumnHidden(2, true);
    }
}


