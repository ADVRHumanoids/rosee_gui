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
        RCLCPP_ERROR_STREAM (_node->get_logger(),"urdfFilePath passed is an empty string!");
        return;
    }
    
    _urdfModel = urdf::parseURDF(_urdfFile);
}
**/

RobotDescriptionHandler::RobotDescriptionHandler(const rclcpp::Node::SharedPtr node, std::string urdfFile, std::string srdfFile) {

    _node = node;
    
    this->_urdfFile = urdfFile;
    this->_srdfFile = srdfFile;
    
    if(_urdfFile.empty())
    {
        RCLCPP_ERROR_STREAM (_node->get_logger(),"urdfFile passed is an empty string!");
        return;
    }
    
    if(_srdfFile.empty())
    {
        RCLCPP_ERROR_STREAM (_node->get_logger(),"srdfFile passed is an empty string!");
        return;
    }
    
    _urdfModel = urdf::parseURDF(_urdfFile);
    
    //initString take as 2nd arg the xml content of the file. 
    //initFile takes instead the flename. We have the file xml content in the param server
    _srdfModel = std::make_shared<srdf::Model>();
    _srdfModel->initString ( *_urdfModel, _srdfFile );
    
    look4ActuatedJoints();

}

std::shared_ptr<srdf::Model> RobotDescriptionHandler::getSrdfModel() {
    
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

std::map<std::string, ROSEE::JointActuatedType> RobotDescriptionHandler::getActuatedTypeJointsMap() {
    
    return _actuatedTypeJointsMap;
}

std::vector<std::string> RobotDescriptionHandler::getJointsByActuatedType(ROSEE::JointActuatedType type) {
    
     switch (type) {
        case ROSEE::JointActuatedType::ACTIVE : 
            return _activeJoints;
            
        case ROSEE::JointActuatedType::PASSIVE : 
            return _passiveJoints;
            
        case ROSEE::JointActuatedType::MIMIC : 
            return _mimicJoints;
            
        default:
            RCLCPP_ERROR_STREAM (_node->get_logger(),"[RobotDescriptionHandler::getJointsByActuatedType] type '" << type 
             << "' not recognized, returning an empty vector");
            return std::vector<std::string>();
     }
}

std::vector <std::string> RobotDescriptionHandler::getAllJoints() {
    return _allJoints;
}



void RobotDescriptionHandler::look4ActuatedJoints(){
    
    
    for (auto it : _urdfModel->joints_) { //this contains all joints
        
        if (it.second->type == urdf::Joint::FIXED) {
            continue; //we do not want to display status of fixed
        }
        
        if (it.second->mimic == nullptr) { //not a mimic joint...
            
            _actuatedTypeJointsMap[it.second->name] = ROSEE::JointActuatedType::ACTIVE;
            
        } else {
           
            _actuatedTypeJointsMap[it.second->name] = ROSEE::JointActuatedType::MIMIC;
        }
    }
    
    //now we look for passive. If any found, the entry in the map will be overwritten with
    //the value PASSIVE
    for (auto it : _srdfModel->getPassiveJoints()) {
        _actuatedTypeJointsMap[it.name_] = ROSEE::JointActuatedType::PASSIVE;
    }
    
    //we fill the vectors that can be useful
    for (auto it : _actuatedTypeJointsMap) {
        
        switch (it.second) {
            case ROSEE::JointActuatedType::ACTIVE : 
                _activeJoints.push_back(it.first);
                break;
                
            case ROSEE::JointActuatedType::PASSIVE : 
                _passiveJoints.push_back(it.first);
                break;
                
            case ROSEE::JointActuatedType::MIMIC : 
                _mimicJoints.push_back(it.first);
                break;
                
            default:
                break;
            
        }
    }
    
    //we also fill a vector containing ALL joints, but in order type, first the active, then
    //the mimic, and then the passive
    _allJoints.insert(_allJoints.end(), _activeJoints.begin(), _activeJoints.end());
    _allJoints.insert(_allJoints.end(), _mimicJoints.begin(), _mimicJoints.end());
    _allJoints.insert(_allJoints.end(), _passiveJoints.begin(), _passiveJoints.end());
}



