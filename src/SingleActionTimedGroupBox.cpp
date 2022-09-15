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

#include <rosee_gui/SingleActionTimedGroupBox.h>

SingleActionTimedGroupBox::SingleActionTimedGroupBox (const rclcpp::Node::SharedPtr node, 
    const rosee_msg::msg::GraspingAction graspingAction, QWidget* parent) : QGroupBox(parent) {
    
    _node = node;
    
    unsigned int nInner = graspingAction.inner_actions.size();
        
    if (nInner == 0) {
        RCLCPP_ERROR_STREAM (_node->get_logger(),"[ActionTimed not valid: no inner actions selected");
        throw "";
    }
    
    if (nInner != graspingAction.before_time_margins.size() ||
        nInner != graspingAction.after_time_margins.size() ) {
        RCLCPP_ERROR_STREAM (_node->get_logger(),"[ERROR SingleActionTimedGroupBox costructor] different size of innerNames and innerTimeMargins: " 
        << nInner << " and " << graspingAction.before_time_margins.size() <<
        "(before marg), and " << graspingAction.after_time_margins.size()
        << "(after marg)");
        throw "";
    } 
    
    this->actionName = graspingAction.action_name;
    this->rosMsgSeq = 0;

    setRosActionClient();
                                          
    this->setMinimumSize(120*nInner,100);
    this->setMaximumSize(400*nInner,400);

    grid = new QGridLayout;
    
    windowLabel = new QLabel (QString::fromStdString(graspingAction.action_name));
    windowLabel->setAlignment(Qt::AlignCenter);
    windowLabel->setStyleSheet("QLabel { font-size : 25px }");
    grid->addWidget(windowLabel, 0, 0, 1, nInner);
    
    send_button = new QPushButton ( "SEND", this );
    send_button->setGeometry ( 100, 140, 100, 40 );
    QObject::connect(send_button, SIGNAL ( clicked()), this, SLOT (sendBtnClicked() ) );
    grid->addWidget(send_button, 3, 0, 1, nInner);
    
    for (int i =0; i<nInner; i++) {
        ActionTimedElement* element = 
            new ActionTimedElement(graspingAction.inner_actions.at(i),
                                   graspingAction.before_time_margins.at(i),
                                   graspingAction.after_time_margins.at(i), this);
        //we set object name so we can retrieve later with this->getChild(name)
        //TODO BUG problem when we have more action with same name.. can happen if they are
        // of different type (eg 2 primitive cant have same name, but theoretically a primitive and
        // a composed yes
        element->setObjectName(QString::fromStdString(graspingAction.inner_actions.at(i)));
        grid->addWidget(element, 2, i );
    }
    
    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}

void SingleActionTimedGroupBox::sendBtnClicked() {

    RCLCPP_INFO_STREAM(_node->get_logger(), "[SingleActionTimedGroupBox " << actionName << "] Sending ROS message..." ) ;
    sendActionRos();
    RCLCPP_INFO_STREAM(_node->get_logger(), "[SingleActionTimedGroupBox " << actionName << "] ... sent" ) ;

}

void SingleActionTimedGroupBox::setRosActionClient() {
    
    action_client = rclcpp_action::create_client<rosee_msg::action::ROSEECommand>(
            _node,
            "/action_command");

        //(*nh, "/ros_end_effector/action_command", false);
        //false because we handle the thread
}

void SingleActionTimedGroupBox::sendActionRos () {
    
    using namespace std::placeholders;

    auto goal_msg = rosee_msg::action::ROSEECommand::Goal();
    goal_msg.goal_action.seq = rosMsgSeq++ ;
    goal_msg.goal_action.stamp = rclcpp::Clock().now();
    //goal_msg.goal_action.percentage not used for timed
    goal_msg.goal_action.action_name = actionName;
    goal_msg.goal_action.action_type = ROSEE::Action::Type::Timed ;
    goal_msg.goal_action.action_primitive_type = ROSEE::ActionPrimitive::Type::None ;
    //goal_msg.goal_action.selectable_items left empty 

    
    auto send_goal_options = rclcpp_action::Client<rosee_msg::action::ROSEECommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SingleActionTimedGroupBox::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SingleActionTimedGroupBox::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&SingleActionTimedGroupBox::result_callback, this, _1);
      
    action_client->async_send_goal(goal_msg, send_goal_options);
    
}


void SingleActionTimedGroupBox::goal_response_callback(
    std::shared_future<GoalHandleGraspingActionROS::SharedPtr> future) {
    
    RCLCPP_INFO_STREAM(_node->get_logger(),"[SingleActionTimedGroupBox " << actionName << "] Goal just went active");
    
}

void SingleActionTimedGroupBox::feedback_callback(
    GoalHandleGraspingActionROS::SharedPtr,
    const std::shared_ptr<const rosee_msg::action::ROSEECommand::Feedback> feedback) {
    
    RCLCPP_INFO_STREAM(_node->get_logger(),"[SingleActionTimedGroupBox " << actionName << "] Got Feedback: " <<
        feedback->action_name_current << " , "  << feedback->completation_percentage);    

    ActionTimedElement* el = this->findChild <ActionTimedElement*> (
            QString::fromStdString(feedback->action_name_current));
    
    if (el == NULL ){ 
        RCLCPP_ERROR_STREAM (_node->get_logger(),feedback->action_name_current << " not a child of this action layout");        
        RCLCPP_ERROR_STREAM (_node->get_logger(),"Child Are:");
        for (auto child : findChildren<QWidget *>()) {
            if (! child->objectName().isNull() ) {
                RCLCPP_WARN_STREAM(_node->get_logger(),  qPrintable(child->objectName()) );

            } else {
                RCLCPP_WARN_STREAM(_node->get_logger(), "nullName" );
            }
        }
    }
    el->setProgressBarValue(feedback->completation_percentage);
   
}

void SingleActionTimedGroupBox::result_callback(
    const GoalHandleGraspingActionROS::WrappedResult & result) {
    
    RCLCPP_INFO_STREAM(_node->get_logger(),"[SingleActionTimedGroupBox " << actionName << "] Finished");
    //progressBar->setValue(100);
    
}


//TODO or it is better to store the child and not look for them each time?
void SingleActionTimedGroupBox::resetAll() {
    
    QList<ActionTimedElement *> childElements = this->findChildren<ActionTimedElement *>();
    
    for (auto it : childElements) {
        it->resetAll();
    }
}
