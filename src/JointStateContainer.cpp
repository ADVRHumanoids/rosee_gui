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
 
    this->robotDescriptionHandler = robotDescriptionHandler;
                                              
    jointStateTable = new JointStateTable(nh, robotDescriptionHandler, parent);
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
    
    QCheckBox* actuatedBox = new QCheckBox ( "actuated joints");
    actuatedBox->setToolTip("Check to show the state of actuated joints");
    actuatedBox->setChecked(true);
    QObject::connect( actuatedBox, SIGNAL (stateChanged(int)), 
                      this,   SLOT(showActuatedJoints(int)) );
    
    QCheckBox* mimicBox = new QCheckBox ( "mimic joints");
    mimicBox->setToolTip("Check to show the state of mimic joints");
    mimicBox->setChecked(true);
    QObject::connect( mimicBox, SIGNAL (stateChanged(int)), 
                      this,   SLOT(showMimicJoints(int)) );
    
    QCheckBox* passiveBox = new QCheckBox ( "passive joints");
    passiveBox->setToolTip("Check to show the state of passive joints");
    passiveBox->setChecked(true);
    QObject::connect( passiveBox, SIGNAL (stateChanged(int)), 
                      this,   SLOT(showPassiveJoints(int)) );
    
    jointStateTableOptions->addWidget(posBox, 0, 0);
    jointStateTableOptions->addWidget(velBox, 0, 1);
    jointStateTableOptions->addWidget(effBox, 0, 2);
    jointStateTableOptions->addWidget(actuatedBox, 1, 0);
    jointStateTableOptions->addWidget(mimicBox, 1, 1);
    jointStateTableOptions->addWidget(passiveBox, 1, 2);

    this->addWidget(jointStateTable);
    this->addLayout(jointStateTableOptions);
    
}

void JointStateContainer::showActuatedJoints(int state) {
    
    auto actuatedJointMap = robotDescriptionHandler->getActuatedJointsMap();
    
    for (int row = 0; row < jointStateTable->rowCount(); row++) {
        
        std::string jointName = jointStateTable->verticalHeaderItem(row)->text().toStdString();
        
        auto mapEl = actuatedJointMap.find(jointName);
        
        if (mapEl == actuatedJointMap.end()) {
            
            ROS_ERROR_STREAM (__func__ << " '" << jointName << "' not found in the joint map" <<
               " of the robotDescriptionHandler, strange error happened" );
            return;
        }
        
        if (mapEl->second == ROSEE::JointActuatedType::ACTUATED) {
            
            (state == Qt::Checked) ? jointStateTable->setRowHidden(row, false) : 
                                     jointStateTable->setRowHidden(row, true);
        }
    }
}

void JointStateContainer::showMimicJoints(int state) {
    
    auto actuatedJointMap = robotDescriptionHandler->getActuatedJointsMap();
    
    for (int row = 0; row < jointStateTable->rowCount(); row++) {
        
        std::string jointName = jointStateTable->verticalHeaderItem(row)->text().toStdString();
        
        auto mapEl = actuatedJointMap.find(jointName);
        
        if (mapEl == actuatedJointMap.end()) {
            
            ROS_ERROR_STREAM (__func__ << " '" << jointName << "' not found in the joint map" <<
               " of the robotDescriptionHandler, strange error happened" );
            return;
        }
        
        if (mapEl->second == ROSEE::JointActuatedType::MIMIC) {
            
            (state == Qt::Checked) ? jointStateTable->setRowHidden(row, false) : 
                                     jointStateTable->setRowHidden(row, true);
        }
    }
}

void JointStateContainer::showPassiveJoints(int state) {
    
    auto actuatedJointMap = robotDescriptionHandler->getActuatedJointsMap();
    
    for (int row = 0; row < jointStateTable->rowCount(); row++) {
        
        std::string jointName = jointStateTable->verticalHeaderItem(row)->text().toStdString();
        
        auto mapEl = actuatedJointMap.find(jointName);
        
        if (mapEl == actuatedJointMap.end()) {
            
            ROS_ERROR_STREAM (__func__ << " '" << jointName << "' not found in the joint map" <<
               " of the robotDescriptionHandler, strange error happened" );
            return;
        }
        
        if (mapEl->second == ROSEE::JointActuatedType::PASSIVE) {
            
            (state == Qt::Checked) ? jointStateTable->setRowHidden(row, false) : 
                                     jointStateTable->setRowHidden(row, true);
        }
    }
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


