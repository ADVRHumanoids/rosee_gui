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
    
    for (auto primitive : primitivesAggregatedAvailableMsg) {
        
        std::map<std::string, std::vector<std::string>> pairedElementMap ;
        if (primitive.max_selectable == 2 ) {
            
            // get from service that provide info of which element can be paired
            // so in the gui we disable the not pairable checkboxes if one is checked
            pairedElementMap = getPairMap(primitive.action_name, primitive.selectable_names);
        }
        
        SingleActionBoxesGroupBox* singleActionBoxesGroupBox;

        if (pairedElementMap.size() > 0 ) {
            
            singleActionBoxesGroupBox = 
                new SingleActionBoxesGroupBox(nh, primitive, pairedElementMap, this) ;  
                
        } else {

            singleActionBoxesGroupBox = 
                new SingleActionBoxesGroupBox(nh, primitive, this) ; 
        }

        grid->addWidget (singleActionBoxesGroupBox, rowCol/4, rowCol%4);
        rowCol++;
        
    }
    
    for (auto generic : genericsAvailableMsg) {
        SingleActionGroupBox* singleActionGroupBox = new SingleActionGroupBox(nh, generic.action_name, (ROSEE::Action::Type)generic.action_type, this);
            grid->addWidget (singleActionGroupBox, rowCol/4, rowCol%4);
            rowCol++;
    } 
    
    for (auto timed : timedsAvailableMsg) {
        
        SingleActionTimedGroupBox* singleActionTimedGroupBox = new SingleActionTimedGroupBox(nh, timed, this);
        grid->addWidget(singleActionTimedGroupBox, rowCol/4, rowCol%4, 1, timed.inner_actions.size());
        
        //timed action occupy more space in the grid... -1 because +1 increment
        //is already present at the end of this switch
        rowCol += (timed.inner_actions.size());

    }

    //special last button to reset all widget and send 0 pos to all joints
    resetButton = new QPushButton("Reset GUI", this);
    resetButton->setMinimumSize(150,50);
    resetButton->setMaximumSize(200,70);
    resetButton->setAutoFillBackground(true);
    QPalette palette = resetButton->palette();
    resetButton->setStyleSheet("QPushButton {background-color: red; color: black;}");
    resetButton->setPalette(palette);
    connect (resetButton, SIGNAL (clicked()), this, SLOT (resetButtonClicked()));
    
    //we place the button spanning the available space in the row (so, 4-(rowCol/4) )
    grid->addWidget(resetButton, rowCol/4, rowCol%4, 1, 4-(rowCol/4), Qt::AlignCenter);
    
    this->setLayout(grid);
    
}


void ContainerActionGroupBox::getInfoServices() {
    
    std::string primitiveAggregatedSrvName, graspingActionsSrvName;
    nh->param<std::string>("/rosee/grasping_action_srv_name", graspingActionsSrvName, 
                           "grasping_actions_available");
    
    nh->param<std::string>("/rosee/primitive_aggregated_srv_name", primitiveAggregatedSrvName, 
                           "primitives_aggregated_available");
    
    graspingActionsSrvName = "/ros_end_effector/" + graspingActionsSrvName;
    primitiveAggregatedSrvName = "/ros_end_effector/" + primitiveAggregatedSrvName;
    
    //wait for infinite (-1) for the service
    ros::service::waitForService(graspingActionsSrvName, -1);
    ROS_INFO_STREAM ("... " << graspingActionsSrvName << " service found, I will call it");
    
    ros::service::waitForService(primitiveAggregatedSrvName, -1);
    ROS_INFO_STREAM ("... " << primitiveAggregatedSrvName << " service found, I will call it");
        
    //for primitives, we call the primitiveAggregatedSrvName service
    rosee_msg::GraspingPrimitiveAggregatedAvailable primitiveAggregatedSrv;
    if (ros::service::call (primitiveAggregatedSrvName, primitiveAggregatedSrv)) {
        primitivesAggregatedAvailableMsg = primitiveAggregatedSrv.response.primitives_aggregated;

    } else {
        ROS_ERROR_STREAM (" ros::service::call FAILED for primitives " );
    }
    
    rosee_msg::GraspingActionsAvailable graspingActionSrv;
    graspingActionSrv.request.action_type = 1; //generic & composed
    if (ros::service::call (graspingActionsSrvName, graspingActionSrv)) {
        genericsAvailableMsg = graspingActionSrv.response.grasping_actions;
    } else {
        ROS_ERROR_STREAM (" ros::service::call FAILED for generic and composed" );
    }
    
    graspingActionSrv.request.action_type = 2; //timed
    if (ros::service::call (graspingActionsSrvName, graspingActionSrv)) {
        timedsAvailableMsg = graspingActionSrv.response.grasping_actions;
    } else {
        ROS_ERROR_STREAM (" ros::service::call FAILED for timed" );
    }
    
}


std::map < std::string, std::vector<std::string> > ContainerActionGroupBox::getPairMap( 
    std::string action_name, std::vector<std::string> elements) {
    
    std::map<std::string, std::vector<std::string>> pairedElementMap;

    std::string selectablePairSrvName;
    nh->param<std::string>("/rosee/selectable_finger_pair_info", selectablePairSrvName, 
                           "selectable_finger_pair_info");
    
    selectablePairSrvName = "/ros_end_effector/" + selectablePairSrvName;
    
    
    ROS_INFO_STREAM ("waiting "<< selectablePairSrvName << " service for 5 seconds...");
    if (! ros::service::waitForService(selectablePairSrvName, 5000)) {
        ROS_WARN_STREAM (selectablePairSrvName << " not found");

        return std::map < std::string, std::vector<std::string> >();
    }
    ROS_INFO_STREAM ("..." << selectablePairSrvName << " service found, I will call it");

    rosee_msg::SelectablePairInfo pairInfo;
    pairInfo.request.action_name = action_name;

    for (auto elementName : elements) {
        pairInfo.request.element_name = elementName;
        if (ros::service::call(selectablePairSrvName, pairInfo)) {

            pairedElementMap.insert(std::make_pair(elementName, pairInfo.response.pair_elements) );
            
        } else {
            
            ROS_ERROR_STREAM (selectablePairSrvName << " call failed with " << 
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
