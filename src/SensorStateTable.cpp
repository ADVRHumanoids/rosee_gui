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

#include <rosee_gui/SensorStateTable.h>

SensorStateTable::SensorStateTable(ros::NodeHandle* nh, std::string topicName, QWidget* parent)
    : QTableWidget(0,0, parent) {
       
    initialized = false;    
    setMinimumSize(1000,1000);
    
     //table not editable
    setEditTriggers(QAbstractItemView::NoEditTriggers);
    //scroll on the horizontal by pixel, otherwise bad visualization occur when scrolling
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    
    horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeMode::Stretch);
    horizontalHeader()->setSectionsMovable(true); 
    verticalHeader()->setSectionsMovable(true); 
    
    //TODO take info somewhere
    setColumnCount(3);
    QStringList headerLabels;
    headerLabels.append("Position");
    headerLabels.append("Velocity");
    headerLabels.append("Effort");
    setHorizontalHeaderLabels(headerLabels);
    
    //who is afraid of lambdas and boost::functions ?
    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
    callback = [this, topicName](const topic_tools::ShapeShifter::ConstPtr& msg) -> void
    {
        this->topicCallback(msg, topicName, rosIntroParser) ;
    };
    _subscriber =  nh->subscribe(topicName, 1, callback) ; 
        
        
}


void SensorStateTable::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                   const std::string &topic_name,
                   RosIntrospection::Parser& rosIntroParser) {
    
    const std::string&  datatype   =  msg->getDataType();
    const std::string&  definition =  msg->getMessageDefinition();

    // don't worry if you do this more than once: already registered message are not overwritten.
    rosIntroParser.registerMessageDefinition( topic_name,
                                      RosIntrospection::ROSType(datatype),
                                      definition );
    
    // reuse these opbects to improve efficiency ("static" makes them persistent)
    static std::vector<uint8_t> buffer;
    static std::map<std::string,RosIntrospection::FlatMessage>   flat_containers;

    RosIntrospection::FlatMessage&   flat_container = flat_containers[topic_name];

    // copy raw memory into the buffer
    buffer.resize( msg->size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);


    // deserialize and rename the vectors
    rosIntroParser.deserializeIntoFlatContainer( topic_name, RosIntrospection::Span<uint8_t>(buffer),
                                                 &flat_container, 100);

    if (initialized == false) {

        for (auto it: flat_container.name)
        {
            
            std::cout << it.first.toStdString() << std::endl;
            std::cout << it.second << std::endl;
                    
            QTableWidgetItem* labelWidget = new QTableWidgetItem(QString::fromStdString(it.second));
            //it.first.index_array[0] contains the #, eg  for /js_publisher/joint_states/name.13 is 13
            //we can use it to select the right row of the table
            //if not, it is not a vector of string, so probably it is not a suitable row label.
            //eg, it is the frame id of the header. 
            //If no vector of string are present in the message, the table will have only a single row,
            // the label can be the topic name for example.
            //TODO if mappa size = 0, setta nome row... o no, lascia senza row label
            //
            if (it.first.index_array.size()>0){
                
                setRowCount(rowCount()+1);
                setVerticalHeaderItem(it.first.index_array[0], labelWidget);
                                   
                //create the cell here once, in clbk only the text will be updated
                for (int i =0; i < columnCount(); i++) {
                    this->setItem (it.first.index_array[0], i, new QTableWidgetItem (  ) );
                }
       
                
            } 
            //else, or the information is not relevant to the sensors (eg an header) or 
            //the whole message contain info of only a single sensor (then, it will not have 
            // a vector of string inside). This second case is considered after this
        }
        
        if (rowCount() == 0) {
            //this means that only a sensor is considered.
            setRowCount(rowCount()+1);
                                   
            //create the cell here once, in clbk only the text will be updated
            for (int i =0; i < columnCount(); i++) {
                this->setItem (0, i, new QTableWidgetItem (  ) );
            }
        }
        

        initialized = true;
        
    } else {
        
        for (auto it: flat_container.value)
        {

            const std::string& key = it.first.toStdString();
            const RosIntrospection::Variant& value  = it.second;
            
            //TODO take this from config file
            //find the substring "position"
            if (key.find("position") != std::string::npos) {
                
                if (it.first.index_array.size()>0){

                    item(it.first.index_array[0], 0) -> //TODO col not hardcoded but take position
                        setText(QString::number(value.convert<double>(), 'f', 2) );
                        
                } else {
                    item(0, 0) -> //TODO col not hardcoded but take position
                        setText(QString::number(value.convert<double>(), 'f', 2) ); 
                }
                
                
            } else if (key.find("velocity") != std::string::npos) {
                
                if (it.first.index_array.size()>0){

                    this->item(it.first.index_array[0], 1) -> //TODO col not hardcoded but take position
                        setText(QString::number(value.convert<double>(), 'f', 2) );
                        
                } else {
                    item(0, 0) -> //TODO col not hardcoded but take position
                        setText(QString::number(value.convert<double>(), 'f', 2) ); 
                }
                
                
            } else if (key.find("effort") != std::string::npos) {
                
                if (it.first.index_array.size()>0){

                    this->item(it.first.index_array[0], 2) -> //TODO col not hardcoded but take position
                        setText(QString::number(value.convert<double>(), 'f', 2) );
                        
                } else {
                    item(0, 0) -> //TODO col not hardcoded but take position
                        setText(QString::number(value.convert<double>(), 'f', 2) ); 
                }
                
            }  
        }
    }
}
