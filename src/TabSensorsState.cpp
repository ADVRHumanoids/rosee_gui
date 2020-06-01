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

TabSensorsState::TabSensorsState ( ros::NodeHandle* nh, std::vector <std::string> topicNames,
                                   QWidget* parent ) : QWidget(parent) {
                                           
    RosIntrospection::Parser rosIntroParser;
    
    //better to create all the tables before... so message does not arrive while table still need 
    //to be created
    for (const auto name: topicNames) {
        
        _tables[name] = new QTableWidget(this);
    }
            
    for (const auto topic_name : topicNames)
    {
        //who is afraid of lambdas and boost::functions ?
        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
        callback = [&rosIntroParser, topic_name, this](const topic_tools::ShapeShifter::ConstPtr& msg) -> void
        {
            this->topicCallback(msg, topic_name, rosIntroParser) ;
        };
        _subscribers.push_back( nh->subscribe(topic_name, 10, callback) );
    }
    
    //while (ros::ok()) {
    //    ros::spinOnce();
    //}

    
}

void TabSensorsState::topicCallback(const RosMsgParser::ShapeShifter& msg,
                   const std::string &topic_name,
                   RosMsgParser::ParsersCollection& rosIntroParsers) {
    

    // you must register the topic definition.
    //  Don't worry, it will not be done twice
    rosIntroParsers.registerParser(topic_name, msg);

    auto deserialized_msg = rosIntroParsers.deserialize(topic_name, msg);

    // Print the content of the message
    printf("--------- %s ----------\n", topic_name.c_str() );
    
    //CANT understand what is its purpose...
    //for (auto it: renamed_values)
    //{
    //    const std::string& key = it.first;
    //    const Variant& value   = it.second;
    //    printf(" %s = %f\n", key.c_str(), value.convert<double>() );
   // }


    for (auto it: deserialized_msg->flat_msg.name)
    {
        
        //     const std::string& key = it.first.toStdString();
        //     const std::string& value = it.second;
        //     printf(" %s = %s\n", key.c_str(), value.c_str());
        
        _tables[topic_name]->setRowCount(deserialized_msg->flat_msg.name.size());
        
        QTableWidgetItem* labelWidget = new QTableWidgetItem(QString::fromStdString(it.second));
        //it.first.index_array[0] contains the #, eg  for /js_publisher/joint_states/name.13 is 13
        //we can use it to select the right row of the table
        //if not, it is not a vector of string, so probably it is not a suitable row label.
        //eg, it is the frame id of the header. 
        //If no vector of string are present in the message, the table will have only a single row,
        // the label can be the topic name for example.
        //
        if (it.first.index_array.size()>0){
            _tables[topic_name]->setVerticalHeaderItem(it.first.index_array[0], labelWidget);
        }
    }
    
    for (auto it : deserialized_msg->renamed_vals)
    {
        
        const std::string& key = it.first;
        const double value = it.second;
        
        //TODO take this from config file
        if (key.find("position") != std::string::npos) {

            //this is done outside... if we know the labels...
            //QTableWidgetItem* labelWidget = new QTableWidgetItem("position");
            //if _tables[topic_name].l
            
            if (it.first.index_array.size()>0){
                _tables[topic_name]->item(it.first.index_array[0], 0) -> 
                    setText(QString::number(value, 'f', 2) );
            } else {
               _tables[topic_name]->item(0, 0) -> //TODO col not hardcoded but take position
                    setText(QString::number(value, 'f', 2) ); 
            }
            
        } else if (key.find("velocity") != std::string::npos) {
        
            
        } else if (key.find("effort") != std::string::npos) {
            
        }
        
        
        
    }
    
}
