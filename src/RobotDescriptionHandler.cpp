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

#include <rosee_gui/RobotDescriptionHandler.h>

/**
RobotDescriptionHandler::RobotDescriptionHandler(std::string urdfFile) {
    
    this->_urdfFile = urdfFile;
    this->_srdfFile = "";
    
    if(_urdfFile.empty())
    {
        ROS_ERROR_STREAM("urdfFilePath passed is an empty string!");
        return;
    }
    
    _urdfModel = urdf::parseURDF(_urdfFile);
}
**/

RobotDescriptionHandler::RobotDescriptionHandler(std::string urdfFile, std::string srdfFile) {
    
    this->_urdfFile = urdfFile;
    this->_srdfFile = srdfFile;
    
    if(_urdfFile.empty())
    {
        ROS_ERROR_STREAM("urdfFile passed is an empty string!");
        return;
    }
    
    if(_srdfFile.empty())
    {
        ROS_ERROR_STREAM("srdfFile passed is an empty string!");
        return;
    }
    
    _urdfModel = urdf::parseURDF(_urdfFile);
    
    //initString take as 2nd arg the xml content of the file. 
    //initFile takes instead the flename. We have the file xml content in the param server
    _srdfModel = boost::make_shared<srdf::Model>();
    _srdfModel->initString ( *_urdfModel, _srdfFile );
    
    look4ActuatedJoints();

}

srdf::ModelSharedPtr RobotDescriptionHandler::getSrdfModel() {
    
    return _srdfModel;
}

urdf::ModelInterfaceSharedPtr RobotDescriptionHandler::getUrdfModel() {
    
    return _urdfModel;
}

std::string RobotDescriptionHandler::getUrdfFile() {
    
    return _urdfFile;
}

std::string RobotDescriptionHandler::getSrdfFile() {
    
    return _srdfFile;

}

std::map<std::string, ROSEE::JointActuatedType> RobotDescriptionHandler::getActuatedJointsMap() {
    
    return _actuatedJointsMap;
}



void RobotDescriptionHandler::look4ActuatedJoints(){
    
    
    for (auto it : _urdfModel->joints_) { //this contains all joints
        
        if (it.second->type == urdf::Joint::FIXED) {
            continue; //we do not want to display status of fixed
        }
        
        if (it.second->mimic == nullptr) { //not a mimic joint...
            
            _actuatedJointsMap[it.second->name] = ROSEE::JointActuatedType::ACTUATED;
            
        } else {
           
            _actuatedJointsMap[it.second->name] = ROSEE::JointActuatedType::MIMIC;
        }
    }
    
    //now we look for passive. If any found, the entry in the map will be overwritten with
    //the value PASSIVE
    for (auto it : _srdfModel->getPassiveJoints()) {
        _actuatedJointsMap[it.name_] = ROSEE::JointActuatedType::PASSIVE;
    }
    
    
}



