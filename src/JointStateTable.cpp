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

#include <rosee_gui/JointStateTable.h>
#include <qheaderview.h> //to modify font size

JointStateTable::JointStateTable (ros::NodeHandle* nh, int rows, int columns, QWidget *parent) :
    QTableWidget(rows, columns, parent) {
        
    if (setJointStateSub(nh)) {
        this->setMinimumSize(600,600);
        
        //table not editable
        setEditTriggers(QAbstractItemView::NoEditTriggers);
        
        this->setColumnCount(3); //pos vel effort

        QStringList headerLabels;
        headerLabels.append("Position");
        headerLabels.append("Velocity");
        headerLabels.append("Effort");
        setHorizontalHeaderLabels(headerLabels);
        
        QFont verticalFont = this->verticalHeader()->font();
        QFont horizontalFont = this->horizontalHeader()->font();
        
        verticalFont.setPointSize( 8 );
        this->verticalHeader()->setFont( verticalFont );
        
        horizontalFont.setPointSize(8);
        this->horizontalHeader()->setFont(horizontalFont);
        
        //scroll on the horizontal by pixel, otherwise bad visualization occur when scrolling
        setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        
        

    } else {
        //TODO ERROR
    }
        
}

bool JointStateTable::setJointStateSub(ros::NodeHandle* nh) {
    
    
    //to get joint state from gazebo, if used
    std::string jsTopic;
    nh->param<std::string>("/rosee/joint_states_topic", jsTopic, "/ros_end_effector/joint_states");
    
    ROS_INFO_STREAM ( "Getting joint pos from '" << jsTopic << "'" );
    
    jointPosSub = nh->subscribe (jsTopic, 1, &JointStateTable::jointStateClbk, this);
}

void JointStateTable::jointStateClbk(const sensor_msgs::JointStateConstPtr& msg) {
    
    //in the callback we store in member the joint state. Then we will update in the gui, but not here
    //TODO set the row and the joint names only once, and not in callback? 
    //but if the message change the joint names? or some joint names at some point are not present?
    this->setRowCount(msg->name.size());
    QStringList jointNamesLabels;
    jointNamesLabels.reserve (msg->name.size());
    for (int i = 0; i < msg->name.size(); i++){
        this->setItem (i, 0, new QTableWidgetItem ( QString::number(msg->position.at(i), 'f', 2) ) );
        this->setItem (i, 1, new QTableWidgetItem ( QString::number(msg->velocity.at(i), 'f', 2) ) );
        this->setItem (i, 2, new QTableWidgetItem ( QString::number(msg->effort.at(i)  , 'f', 2) ) );
        jointNamesLabels.insert(i, QString::fromStdString(msg->name.at(i)));
    }
    
    setVerticalHeaderLabels(jointNamesLabels);
    
}

