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
        grid->addWidget(element, 2, i );
    }
    
    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}

void ActionTimedLayout::sendBtnClicked() {

    ROS_WARN_STREAM ( "TODO, SEND ROS MESSAGE TO TIMED TOPIC" );
}
