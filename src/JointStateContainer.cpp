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
    
                                              
    activeJointStateTable = new JointStateTable(
        nh, 
        robotDescriptionHandler->getJointsByActuatedType(ROSEE::JointActuatedType::ACTIVE), 
        parent);
    mimicJointStateTable = new JointStateTable(
        nh, 
        robotDescriptionHandler->getJointsByActuatedType(ROSEE::JointActuatedType::MIMIC), 
        parent);
    passiveJointStateTable = new JointStateTable(
        nh, 
        robotDescriptionHandler->getJointsByActuatedType(ROSEE::JointActuatedType::PASSIVE), 
        parent);

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
    
    QCheckBox* actuatedBox = new QCheckBox ( "active joints");
    actuatedBox->setToolTip("Check to show the state of active joints");
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

    
    activeJointslabel = new QLabel(activeJointStateTable);
    activeJointslabel->setText("Active Joints");
    this->addWidget(activeJointslabel);
    this->addWidget(activeJointStateTable);


    mimicJointslabel = new QLabel(mimicJointStateTable);
    mimicJointslabel->setText("Mimic Joints");
    mimicJointslabel->setContentsMargins(0,20,0,0);
    this->addWidget(mimicJointslabel);
    this->addWidget(mimicJointStateTable);

    
    passiveJointslabel = new QLabel(passiveJointStateTable);
    passiveJointslabel->setText("Passive Joints");
    passiveJointslabel->setContentsMargins(0,20,0,0);
    this->addWidget(passiveJointslabel);
    this->addWidget(passiveJointStateTable);
    this->addLayout(jointStateTableOptions);
    
    if (robotDescriptionHandler->getJointsByActuatedType(ROSEE::JointActuatedType::ACTIVE).size() == 0) {
        actuatedBox->click();
    } 
    
    if (robotDescriptionHandler->getJointsByActuatedType(ROSEE::JointActuatedType::MIMIC).size() == 0) {
        mimicBox->click();
    } 
    
    if (robotDescriptionHandler->getJointsByActuatedType(ROSEE::JointActuatedType::PASSIVE).size() == 0) {
        passiveBox->click();
    } 
    
    
}

void JointStateContainer::showActuatedJoints(int state) {
    
    if (state == Qt::Checked) {
        activeJointStateTable->setHidden(false);
        activeJointslabel->setHidden(false);
        
    } else {
        activeJointStateTable->setHidden(true);
        activeJointslabel->setHidden(true);     
        
    }
}

void JointStateContainer::showMimicJoints(int state) {
    
    if (state == Qt::Checked) {
        mimicJointStateTable->setHidden(false);
        mimicJointslabel->setHidden(false);
        
    } else {
        mimicJointStateTable->setHidden(true);
        mimicJointslabel->setHidden(true);     
        
    }
}

void JointStateContainer::showPassiveJoints(int state) {
    
    if (state == Qt::Checked) {
        passiveJointStateTable->setHidden(false);
        passiveJointslabel->setHidden(false);
        
    } else {
        passiveJointStateTable->setHidden(true);
        passiveJointslabel->setHidden(true);     
        
    }
}


void JointStateContainer::showPositionCol(int state) {
    
    if (state == Qt::Checked) {
        activeJointStateTable->setColumnHidden(0, false);
        passiveJointStateTable->setColumnHidden(0, false);
        mimicJointStateTable->setColumnHidden(0, false);
        
    } else {
        activeJointStateTable->setColumnHidden(0, true);
        passiveJointStateTable->setColumnHidden(0, true);
        mimicJointStateTable->setColumnHidden(0, true);
    }
}

void JointStateContainer::showVelocityCol(int state) {
    
    if (state == Qt::Checked) {
        activeJointStateTable->setColumnHidden(1, false);
        passiveJointStateTable->setColumnHidden(1, false);
        mimicJointStateTable->setColumnHidden(1, false);
        
    } else {
        activeJointStateTable->setColumnHidden(1, true);
        passiveJointStateTable->setColumnHidden(1, true);
        mimicJointStateTable->setColumnHidden(1, true);
    }
}

void JointStateContainer::showEffortCol(int state) {
    
    if (state == Qt::Checked) {
        activeJointStateTable->setColumnHidden(2, false);
        passiveJointStateTable->setColumnHidden(2, false);
        mimicJointStateTable->setColumnHidden(2, false);
        
    } else {
        activeJointStateTable->setColumnHidden(2, true);
        passiveJointStateTable->setColumnHidden(2, true);
        mimicJointStateTable->setColumnHidden(2, true);
    }
}


