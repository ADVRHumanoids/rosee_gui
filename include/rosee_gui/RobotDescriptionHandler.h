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

#ifndef ROBOTDESCRIPTIONHANDLER_H
#define ROBOTDESCRIPTIONHANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <fstream>


namespace ROSEE {
    //for parsing thing of main package, a joint can be OR passive OR mimic, not both
    //PASSIVE is simply a not actuated joint that other actuated one will not influence
    enum JointActuatedType { ACTIVE, MIMIC, PASSIVE };
}

/**
 * @todo write docs
 */
class RobotDescriptionHandler
{
    
public:
    //RobotDescriptorHandler();
    //RobotDescriptionHandler(std::string urdfFile );
    RobotDescriptionHandler(const rclcpp::Node::SharedPtr node, std::string urdfFile, std::string srdfFile);
    
    urdf::ModelInterfaceSharedPtr getUrdfModel();
    std::shared_ptr<srdf::Model> getSrdfModel();
    std::string getUrdfFile();
    std::string getSrdfFile();
    
    std::map <std::string, ROSEE::JointActuatedType> getActuatedTypeJointsMap();
    std::vector<std::string> getJointsByActuatedType(ROSEE::JointActuatedType type);
    std::vector <std::string> getAllJoints();


    
private:
    
    void look4ActuatedJoints();

    rclcpp::Node::SharedPtr _node;
    std::string _urdfFile;
    std::string _srdfFile;
    urdf::ModelInterfaceSharedPtr _urdfModel;
    std::shared_ptr<srdf::Model> _srdfModel;
    
    std::map <std::string, ROSEE::JointActuatedType> _actuatedTypeJointsMap;
    
    std::vector <std::string> _activeJoints;
    std::vector <std::string> _passiveJoints;
    std::vector <std::string> _mimicJoints;
    std::vector <std::string> _allJoints;
    

};

#endif // ROBOTDESCRIPTIONHANDLER_H
