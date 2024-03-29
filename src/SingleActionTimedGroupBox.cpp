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

SingleActionTimedGroupBox::SingleActionTimedGroupBox (ros::NodeHandle* nh, 
    rosee_msg::GraspingAction graspingAction, QWidget* parent) : QGroupBox(parent) {
    
    unsigned int nInner = graspingAction.inner_actions.size();
        
    if (nInner == 0) {
        ROS_ERROR_STREAM("[ActionTimed not valid: no inner actions selected");
        throw "";
    }
    
    if (nInner != graspingAction.before_time_margins.size() ||
        nInner != graspingAction.after_time_margins.size() ) {
        ROS_ERROR_STREAM("[ERROR SingleActionTimedGroupBox costructor] different size of innerNames and innerTimeMargins: " 
        << nInner << " and " << graspingAction.before_time_margins.size() <<
        "(before marg), and " << graspingAction.after_time_margins.size()
        << "(after marg)");
        throw "";
    } 
    
    this->actionName = graspingAction.action_name;
    this->rosMsgSeq = 0;

    setRosActionClient(nh);
                                          
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

    ROS_INFO_STREAM( "[SingleActionTimedGroupBox " << actionName << "] Sending ROS message..." ) ;
    sendActionRos();
    ROS_INFO_STREAM( "[SingleActionTimedGroupBox " << actionName << "] ... sent" ) ;

}

void SingleActionTimedGroupBox::setRosActionClient(ros::NodeHandle* nh) {
    
    action_client = 
        std::make_shared <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction>>
        (*nh, "/ros_end_effector/action_command", false);
        //false because we handle the thread
}

void SingleActionTimedGroupBox::sendActionRos () {

    rosee_msg::ROSEECommandGoal goal;
    goal.goal_action.seq = rosMsgSeq++ ;
    goal.goal_action.stamp = ros::Time::now();
    //goal.goal_action.percentage not used for timed
    goal.goal_action.action_name = actionName;
    goal.goal_action.action_type = ROSEE::Action::Type::Timed ;
    goal.goal_action.actionPrimitive_type = ROSEE::ActionPrimitive::Type::None ;
    //goal.goal_action.selectable_items left empty 

    action_client->sendGoal (goal, boost::bind(&SingleActionTimedGroupBox::doneCallback, this, _1, _2),
        boost::bind(&SingleActionTimedGroupBox::activeCallback, this), boost::bind(&SingleActionTimedGroupBox::feedbackCallback, this, _1)) ;

}

void SingleActionTimedGroupBox::doneCallback(const actionlib::SimpleClientGoalState& state,
            const rosee_msg::ROSEECommandResultConstPtr& result) {
    
    ROS_INFO_STREAM("[SingleActionTimedGroupBox " << actionName << "] Finished in state "<<  state.toString().c_str());
    //progressBar->setValue(100);
    
}

void SingleActionTimedGroupBox::activeCallback() {
    
    ROS_INFO_STREAM("[SingleActionTimedGroupBox " << actionName << "] Goal just went active");
    
}

void SingleActionTimedGroupBox::feedbackCallback(
    const rosee_msg::ROSEECommandFeedbackConstPtr& feedback) {
    
    ROS_INFO_STREAM("[SingleActionTimedGroupBox " << actionName << "] Got Feedback: " <<
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

//TODO or it is better to store the child and not look for them each time?
void SingleActionTimedGroupBox::resetAll() {
    
    QList<ActionTimedElement *> childElements = this->findChildren<ActionTimedElement *>();
    
    for (auto it : childElements) {
        it->resetAll();
    }
}
