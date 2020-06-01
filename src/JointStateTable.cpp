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
 * limitations under th0e License.
 */

#include <rosee_gui/JointStateTable.h>
#include <qheaderview.h> //to modify font size

JointStateTable::JointStateTable (ros::NodeHandle* nh, 
                                  std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler, QWidget *parent) :
    QTableWidget(0, 0, parent) {
        
    if (setJointStateSub(nh)) {
        //this->setMinimumSize(1,1);
        
        //table not editable
        setEditTriggers(QAbstractItemView::NoEditTriggers);
        //scroll on the horizontal by pixel, otherwise bad visualization occur when scrolling
        setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        
        horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeMode::Stretch);
        horizontalHeader()->setSectionsMovable(true); 
        verticalHeader()->setSectionsMovable(true); 
                
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

        auto actuatedJointMap = robotDescriptionHandler->getActuatedJointsMap();
        this->setRowCount(actuatedJointMap.size());
        
        int row = 0;
        for (auto mapEl : actuatedJointMap) {
            
            QTableWidgetItem* labelWidget = new QTableWidgetItem(QString::fromStdString(mapEl.first));
            
            if (mapEl.second == ROSEE::JointActuatedType::ACTUATED) {
                labelWidget->setBackground(Qt::green);
                
            } else if (mapEl.second == ROSEE::JointActuatedType::MIMIC) {
                labelWidget->setBackground(Qt::yellow);     
                
            } else {
                labelWidget->setBackground(Qt::red);
            }

            setVerticalHeaderItem(row, labelWidget);
            
            //create the cell here once, in clbk only the text will be updated
            this->setItem (row, 0, new QTableWidgetItem (  ) );
            this->setItem (row, 1, new QTableWidgetItem (  ) );
            this->setItem (row, 2, new QTableWidgetItem (  ) );

            
            //we need this to update the correct row in the callback
            jointNameRowMap[mapEl.first] = row;
            row++;
        }

    } else {
        //TODO ERROR
    }
        
}

bool JointStateTable::setJointStateSub(ros::NodeHandle* nh) {
    
    //to get joint state from gazebo, if used
    std::string jsTopic;
    if (nh->getParam("/rosee/joint_states_topic", jsTopic)) {
    
        ROS_INFO_STREAM ( "Getting joint pos from '" << jsTopic << "'" );
        
    } else {
        
        ROS_ERROR_STREAM ( "Param '/rosee/joint_states_topic' not set in parameter server");
        return false;
    }
    
    jointPosSub = nh->subscribe (jsTopic, 1, &JointStateTable::jointStateClbk, this);
}

void JointStateTable::jointStateClbk(const sensor_msgs::JointStateConstPtr& msg) {
       
    for (int i = 0; i < msg->name.size(); i++){
        
        auto mapEl = jointNameRowMap.find(msg->name[i]);
        
        if (mapEl == jointNameRowMap.end()){
            ROS_WARN_STREAM (__func__ << " '" << msg->name[i] << "' not found in the table of joint states" );
        
        } else {
            int row = mapEl->second;
            this->item(row, 0) -> setText(QString::number(msg->position.at(i), 'f', 2) );
            this->item(row, 1) -> setText(QString::number(msg->velocity.at(i), 'f', 2) );
            this->item(row, 2) -> setText(QString::number(msg->effort.at(i), 'f', 2) );
        }
    }
}

