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

#include <rosee_gui/ActionTimedLayout.h>

ActionTimedLayout::ActionTimedLayout (ros::NodeHandle* nh, rosee_msg::ActionInfo actInfo,
                                      QWidget* parent) : QGroupBox(parent) {
    
    if (actInfo.inner_actions.size() == 0) {
        ROS_ERROR_STREAM("[ActionTimed not valid: no inner actions selected");
        throw "";
    }
    
    unsigned int nInner = actInfo.inner_actions.size();
    
    if (nInner != actInfo.before_margins.size() ||
        nInner != actInfo.after_margins.size() ) {
        ROS_ERROR_STREAM("[ERROR ActionTimedLayout costructor] different size of innerNames and innerTimeMargins: " 
        << nInner << " and " << actInfo.before_margins.size() <<
        "(before marg), and " << actInfo.after_margins.size()
        << "(after marg)");
        throw "";
    } 
    
    this->actionName = actInfo.action_name;
    this->rosMsgSeq = 0;

    setRosActionClient(nh, actInfo.ros_action_name);
                                          
    this->setMinimumSize(600,200);

    grid = new QGridLayout;
    
    windowLabel = new QLabel (QString::fromStdString(actInfo.action_name));
    windowLabel->setAlignment(Qt::AlignCenter);
    windowLabel->setStyleSheet("QLabel { font-size : 40px }");
    grid->addWidget(windowLabel, 0, 0, 1, nInner);
    
    send_button = new QPushButton ( "SEND", this );
    send_button->setGeometry ( 100, 140, 100, 40 );
    QObject::connect(send_button, SIGNAL ( clicked()), this, SLOT (sendBtnClicked() ) );
    grid->addWidget(send_button, 3, 0, 1, nInner);
    
    for (int i =0; i<nInner; i++) {
        ActionTimedElement* element = 
            new ActionTimedElement(actInfo.inner_actions.at(i), actInfo.before_margins.at(i),
                                   actInfo.after_margins.at(i), this);
        //we set object name so we can retrieve later with this->getChild(name)
        //TODO BUG problem when we have more action with same name.. can happen if they are
        // of different type (eg 2 primitive cant have same name, but theoretically a primitive and
        // a composed yes
        element->setObjectName(QString::fromStdString(actInfo.inner_actions.at(i)));
        grid->addWidget(element, 2, i );
    }
    
    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}

void ActionTimedLayout::sendBtnClicked() {

    ROS_INFO_STREAM( "[ActionTimedLayout " << actionName << "] Sending ROS message..." ) ;
    sendActionRos();
    ROS_INFO_STREAM( "[ActionTimedLayout " << actionName << "] ... sent" ) ;

}

void ActionTimedLayout::setRosActionClient(ros::NodeHandle* nh, std::string rosActionName) {
    
    action_client = 
        std::make_shared <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction>>
        (*nh, rosActionName, false);
        //false because we handle the thread
}

void ActionTimedLayout::sendActionRos () {

    rosee_msg::ROSEECommandGoal goal;
    goal.goal_action.seq = rosMsgSeq++ ;
    goal.goal_action.stamp = ros::Time::now();
    //goal.goal_action.percentage not used for timed
    goal.goal_action.action_name = actionName;
    //actionLAyout can be generic or composed, but it do not change what we put in type
    //because the server will act on them equally
    goal.goal_action.action_type = ActionType::Timed ;
    goal.goal_action.actionPrimitive_type = PrimitiveType::PrimitiveNone ;
    //goal.goal_action.selectable_items left empty 

    action_client->sendGoal (goal, boost::bind(&ActionTimedLayout::doneCallback, this, _1, _2),
        boost::bind(&ActionTimedLayout::activeCallback, this), boost::bind(&ActionTimedLayout::feedbackCallback, this, _1)) ;

}

void ActionTimedLayout::doneCallback(const actionlib::SimpleClientGoalState& state,
            const rosee_msg::ROSEECommandResultConstPtr& result) {
    
    ROS_INFO_STREAM("[ActionTimedLayout " << actionName << "] Finished in state "<<  state.toString().c_str());
    //progressBar->setValue(100);
    
}

void ActionTimedLayout::activeCallback() {
    
    ROS_INFO_STREAM("[ActionTimedLayout " << actionName << "] Goal just went active");
    
}

void ActionTimedLayout::feedbackCallback(
    const rosee_msg::ROSEECommandFeedbackConstPtr& feedback) {
    
    ROS_INFO_STREAM("[ActionTimedLayout " << actionName << "] Got Feedback: " <<
        feedback->action_name_current << " , "  << feedback->completation_percentage);    

    ActionTimedElement* el = this->findChild <ActionTimedElement*> (
            QString::fromStdString(feedback->action_name_current));
    
    if (el == NULL ){ 
        ROS_ERROR_STREAM (feedback->action_name_current << " not a child of this action layout");        
        ROS_ERROR_STREAM ("Child Are:");
        for (auto child : findChildren<QWidget *>()) {
            if (! child->objectName().isNull() ) {
                ROS_WARN_STREAM (  qPrintable(child->objectName()) );

            } else {
                ROS_WARN_STREAM ( "nullName" );
            }
        }
    }
    el->setProgressBarValue(feedback->completation_percentage);
   
}

