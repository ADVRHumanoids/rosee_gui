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

ContainerActionGroupBox::ContainerActionGroupBox (const rclcpp::Node::SharedPtr node, QWidget* parent) : QGroupBox(parent) {
    
    this->_node = node;
    
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
                new SingleActionBoxesGroupBox(_node, primitive, pairedElementMap, this) ;  
                
        } else {

            singleActionBoxesGroupBox = 
                new SingleActionBoxesGroupBox(_node, primitive, this) ; 
        }

        grid->addWidget (singleActionBoxesGroupBox, rowCol/4, rowCol%4);
        rowCol++;
        
    }
    
    for (auto generic : genericsAvailableMsg) {
        SingleActionGroupBox* singleActionGroupBox = new SingleActionGroupBox(_node, generic.action_name, (ROSEE::Action::Type)generic.action_type, this);
            grid->addWidget (singleActionGroupBox, rowCol/4, rowCol%4);
            rowCol++;
    } 
    
    for (auto timed : timedsAvailableMsg) {
        
        SingleActionTimedGroupBox* singleActionTimedGroupBox = new SingleActionTimedGroupBox(_node, timed, this);
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
//     _node->declare_parameter("/rosee/grasping_action_srv_name", "grasping_actions_available");
//     _node->get_parameter("/rosee/grasping_action_srv_name", graspingActionsSrvName);    
//     
//     _node->declare_parameter("/rosee/primitive_aggregated_srv_name", "primitives_aggregated_available");
//     _node->get_parameter("/rosee/primitive_aggregated_srv_name", primitiveAggregatedSrvName);
    
    
    graspingActionsSrvName = "/grasping_actions_available";
    primitiveAggregatedSrvName = "/primitives_aggregated_available";
    
    rclcpp::Client<rosee_msg::srv::GraspingActionsAvailable>::SharedPtr graspingActionAvailableClient =
        _node->create_client<rosee_msg::srv::GraspingActionsAvailable>(graspingActionsSrvName);
    
    rclcpp::Client<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable>::SharedPtr primitivesAggregatedAvailableClient =
        _node->create_client<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable>(primitiveAggregatedSrvName);
    
    RCLCPP_INFO_STREAM (_node->get_logger(), "Waiting for " << graspingActionsSrvName << " service...");
    RCLCPP_INFO_STREAM (_node->get_logger(), "Waiting for " << primitiveAggregatedSrvName << " service...");
    
        
    //wait for infinite (-1) for the service
    graspingActionAvailableClient->wait_for_service();
    
    RCLCPP_INFO_STREAM (_node->get_logger(), "... " << graspingActionsSrvName << " service found, I will call it");
    
    primitivesAggregatedAvailableClient->wait_for_service();
    RCLCPP_INFO_STREAM (_node->get_logger(), "... " << primitiveAggregatedSrvName << " service found, I will call it");
        
    //for primitives, we call the primitiveAggregatedSrvName service
    auto primitivesAggregatedAvailableRequest = std::make_shared<rosee_msg::srv::GraspingPrimitiveAggregatedAvailable::Request>();

    auto primitivesAggregatedAvailableResult = primitivesAggregatedAvailableClient->async_send_request(primitivesAggregatedAvailableRequest);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(_node, primitivesAggregatedAvailableResult) == rclcpp::FutureReturnCode::SUCCESS)
    {
        primitivesAggregatedAvailableMsg = primitivesAggregatedAvailableResult.get()->primitives_aggregated;
    } else {
        RCLCPP_ERROR_STREAM (_node->get_logger()," ros::service::call FAILED for primitives " );
    } 
    
    //graspingsactions
    auto graspingActionAvailableRequest = std::make_shared<rosee_msg::srv::GraspingActionsAvailable::Request>();
    graspingActionAvailableRequest->action_type = 1; //generic & composed
    auto graspingActionAvailableResult = graspingActionAvailableClient->async_send_request(graspingActionAvailableRequest);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(_node, graspingActionAvailableResult) == rclcpp::FutureReturnCode::SUCCESS)
    {
        genericsAvailableMsg = graspingActionAvailableResult.get()->grasping_actions;
    } else {
        RCLCPP_ERROR_STREAM (_node->get_logger()," ros::service::call FAILED for generic and composed" );
    } 
    
    
    graspingActionAvailableRequest->action_type = 2; //timed
    auto graspingActionAvailableResult2 = graspingActionAvailableClient->async_send_request(graspingActionAvailableRequest);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(_node, graspingActionAvailableResult2) == rclcpp::FutureReturnCode::SUCCESS)
    {
        timedsAvailableMsg = graspingActionAvailableResult2.get()->grasping_actions;
    } else {
        RCLCPP_ERROR_STREAM (_node->get_logger()," ros::service::call FAILED for timed" );
    } 
    
}


std::map < std::string, std::vector<std::string> > ContainerActionGroupBox::getPairMap( 
    std::string action_name, std::vector<std::string> elements) {
    
    std::map<std::string, std::vector<std::string>> pairedElementMap;

    std::string selectablePairSrvName;
//     _node->declare_parameter("/rosee/selectable_finger_pair_info", "selectable_finger_pair_info");
//     _node->get_parameter("/rosee/selectable_finger_pair_info", selectablePairSrvName);
    
    selectablePairSrvName = "/selectable_finger_pair_info";
    
    
    RCLCPP_INFO_STREAM (_node->get_logger(), "waiting "<< selectablePairSrvName << " service for 5 seconds...");
    
    rclcpp::Client<rosee_msg::srv::SelectablePairInfo>::SharedPtr pairInfoClient =
        _node->create_client<rosee_msg::srv::SelectablePairInfo>(selectablePairSrvName);
        
    if (! pairInfoClient->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN_STREAM (_node->get_logger(), selectablePairSrvName << " not found");

        return std::map < std::string, std::vector<std::string> >();
    }
    RCLCPP_INFO_STREAM (_node->get_logger(), "..." << selectablePairSrvName << " service found, I will call it");

    auto pairInfoRequest = std::make_shared<rosee_msg::srv::SelectablePairInfo::Request>();
    pairInfoRequest->action_name = action_name;

    for (auto elementName : elements) {
        pairInfoRequest->element_name = elementName;
        
        auto pairInfoResult = pairInfoClient->async_send_request(pairInfoRequest);     

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(_node, pairInfoResult) == rclcpp::FutureReturnCode::SUCCESS)
        {
            pairedElementMap.insert(std::make_pair(elementName, pairInfoResult.get()->pair_elements) );

        } else {
            RCLCPP_ERROR_STREAM (_node->get_logger(),selectablePairSrvName << " call failed with " << 
                pairInfoRequest->action_name << ", " << pairInfoRequest->element_name <<
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
