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

#include <rosee_gui/TabSensorsState.h>

TabSensorsState::TabSensorsState ( ros::NodeHandle* nh, QWidget* parent ) : QWidget(parent) {
    
    QGridLayout *windowGrid = new QGridLayout;
    
    std::string configFileName;
    
    if (! nh->getParam("/ros_ee_config_path", configFileName) ) {
        ROS_ERROR_STREAM ("[ERROR TabSensorsState]: parameter '/ros_ee_config_path'" <<
            " not loaded on parameter server");
        return;
    }
    
    if (! parseConfigYaml(configFileName)) {
        //TODO print error?
        return;
    }
    
    unsigned int iTable = 0;
    for (const auto opt: options) {
        
        _tables[opt.topicName] = new SensorStateTable(nh, opt, parent);
        
        windowGrid->addWidget(_tables[opt.topicName], iTable/2, iTable%2);

        iTable++;
    }
    
    setLayout(windowGrid);

}

bool TabSensorsState::parseConfigYaml(std::string filename) {
    
    YAML::Node node = YAML::LoadFile(filename);
    
    if ( ! node["rosee_gui"] ) {
        ROS_ERROR_STREAM ("[ERROR TabSensorsState]: Not found the node 'rosee_gui' in the config file '" << filename << "' , or the file is missing" );
            return false;
    }
    
    if (! node["rosee_gui"]["sensors_state_widget"] ) {
        ROS_ERROR_STREAM ("[ERROR TabSensorsState]: Not found the child node 'sensors_state_widget' of 'rosee_gui' in the config file '" << filename);
            return false;    
    }
    
    for ( const auto element : node["rosee_gui"]["sensors_state_widget"] ) {
    //one element for each table to build
    
        
        SensorsStateOption opt;
        opt.topicName = element["topic_name"].as<std::string>();

        for ( const auto col : element["column_names"] ) {
            opt.columnNames.push_back( col.first.as<std::string>() );
        }
        
        if ( element["row_name"] ) {
            opt.rowLabel = element["row_name"].as<std::string>();
        }
        
        options.push_back(opt);
    }
    
    return true;    
}


