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

#include <rosee_gui/ContainerActionGroupBox.h>

ContainerActionGroupBox::ContainerActionGroupBox (ros::NodeHandle* nh, QWidget* parent) : QGroupBox(parent) {
    
    this->nh = nh;
    
    // get (wait) for info from unviersalroseeExecutor about all the action parsed by it
    getInfoServices();
    
    grid = new QGridLayout;
    int rowCol = 0;

    for (auto actInfo: actionInfoVect) {
            
        switch (actInfo.action_type) {
        
        case ROSEE::Action::Type::Primitive :
        {
            if (actInfo.max_selectable == 2) {
                // get (wait) for service that provide info of which element can be paired
                // so in the gui we disable the not pairable checkboxes if one is checked
                std::map<std::string, std::vector<std::string>> pairedElementMap = 
                    getPairMap(actInfo.action_name, actInfo.selectable_names);
                
                SingleActionBoxesGroupBox* singleActionBoxesGroupBox;
                if (pairedElementMap.size() != 0) {
                    singleActionBoxesGroupBox = 
                    new SingleActionBoxesGroupBox(nh, actInfo, pairedElementMap, this) ;
                    
                } else {
                    //version without disabling the not pairable checkboxes
                    singleActionBoxesGroupBox = 
                    new SingleActionBoxesGroupBox(nh, actInfo, this) ; 
                    
                }
                grid->addWidget (singleActionBoxesGroupBox, rowCol/4, rowCol%4);
                
            } else {
                SingleActionBoxesGroupBox* singleActionBoxesGroupBox;
                singleActionBoxesGroupBox = 
                    new SingleActionBoxesGroupBox(nh, actInfo, this) ; 
                grid->addWidget (singleActionBoxesGroupBox, rowCol/4, rowCol%4);
            }

            break;
        }
        case ROSEE::Action::Type::Generic : // same thing as composed
        case ROSEE::Action::Type::Composed :
        {
            SingleActionGroupBox* singleActionGroupBox = new SingleActionGroupBox(nh, actInfo, this);
            grid->addWidget (singleActionGroupBox, rowCol/4, rowCol%4);
            break;
        }
        case ROSEE::Action::Type::Timed : 
        {
            SingleActionTimedGroupBox* timed = new SingleActionTimedGroupBox(nh, actInfo, this);
            grid->addWidget(timed, rowCol/4, rowCol%4, 1, actInfo.inner_actions.size());
            
            //timed action occupy more space in the grid... -1 because +1 increment
            //is already present at the end of this switch
            rowCol += (actInfo.inner_actions.size()-1);

            break;
        }
        case ROSEE::Action::Type::None :
        {
            
            ROS_ERROR_STREAM ("GUI ERROR, type NONE received for action " << actInfo.action_name);
            throw "";
            break;
        }
        default : {
            ROS_ERROR_STREAM ("GUI ERROR, not recognized type " << actInfo.action_type
            << " received for action " << actInfo.action_name);
            throw "";
        }
        } 
        
        rowCol++;
    }

    //special last button to reset all widget and send 0 pos to all joints
    resetButton = new QPushButton("Reset GUI", this);
    resetButton->setMinimumSize(50,50);
    resetButton->setAutoFillBackground(true);
    QPalette palette = resetButton->palette();
    resetButton->setStyleSheet("QPushButton {background-color: red; color: black;}");
    resetButton->setPalette(palette);
    connect (resetButton, SIGNAL (clicked()), this, SLOT (resetButtonClicked()));
    
    grid->addWidget(resetButton, rowCol/4, rowCol%4);
    
    this->setLayout(grid);
    
}


void ContainerActionGroupBox::getInfoServices() {
    
    ros::service::waitForService("ros_end_effector/ActionsInfo"); //blocking infinite wait, it also print
    
    rosee_msg::ActionsInfo actionsInfo;

    if (ros::service::call ("ros_end_effector/ActionsInfo", actionsInfo)) {
        actionInfoVect = actionsInfo.response.actionsInfo;
    } else {
        ROS_ERROR_STREAM (" ros::service::call FAILED " );
    }
    
}


std::map < std::string, std::vector<std::string> > ContainerActionGroupBox::getPairMap( 
    std::string action_name, std::vector<std::string> elements) {
    
    std::map<std::string, std::vector<std::string>> pairedElementMap;
    
    ROS_INFO_STREAM ("waiting for ros_end_effector/SelectablePairInfo service for 5 seconds...");
    if (! ros::service::waitForService("ros_end_effector/SelectablePairInfo", 5000)) {
        ROS_WARN_STREAM ("ros_end_effector/SelectablePairInfo not found");
        return std::map < std::string, std::vector<std::string> >();
    }
    ROS_INFO_STREAM ("... service found, I will call it");

    rosee_msg::SelectablePairInfo pairInfo;
    pairInfo.request.action_name = action_name;

    for (auto elementName : elements) {
        pairInfo.request.element_name = elementName;
        if (ros::service::call("ros_end_effector/SelectablePairInfo", pairInfo)) {
            
            pairedElementMap.insert(std::make_pair(elementName, pairInfo.response.pair_elements) );
            
        } else {
            ROS_ERROR_STREAM ("ros_end_effector/SelectablePairInfo call failed with " << 
                pairInfo.request.action_name << ", " << pairInfo.request.element_name <<
                " as request");
            return std::map < std::string, std::vector<std::string> >();

        }
    }
    return pairedElementMap;
}


//TODO or it is better to store the child and not look for them each time?
void ContainerActionGroupBox::resetButtonClicked() {
    
    auto singleActionGroupBoxs = this->findChildren<SingleActionGroupBox *>();

    for (auto it : singleActionGroupBoxs) {
        it->resetAll();
    }
    
    auto singleActionTimedGros = this->findChildren<SingleActionTimedGroupBox *>();
    
    for (auto it : singleActionTimedGros) {

        it->resetAll();
    }
    
    
}
