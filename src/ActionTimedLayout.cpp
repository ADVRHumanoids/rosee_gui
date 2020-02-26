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

ActionTimedLayout::ActionTimedLayout (std::string actionName, 
                                      std::vector<std::string> innerActionNames,
                                      std::vector<std::pair<double,double>> innerTimeMargins,
                                      QWidget* parent) : QGroupBox(parent) {
    
    if (innerActionNames.size() != innerTimeMargins.size() ) {
        std::cerr << "[ERROR ActionTimedLayout costructor] different size of innerNames and innerTimeMargins: " << innerActionNames.size() << " and " << innerTimeMargins.size() << std::endl;
        throw "";
    }                                                        
                                          
    this->setMinimumSize(600,200);

    grid = new QGridLayout;
    
    unsigned int nInner = innerActionNames.size();
    
    windowLabel = new QLabel (QString::fromStdString(actionName));
    windowLabel->setAlignment(Qt::AlignCenter);
    windowLabel->setStyleSheet("QLabel { font-size : 40px }");
    grid->addWidget(windowLabel, 0, 0, 1, nInner);
    
    send_button = new QPushButton ( "SEND", this );
    send_button->setGeometry ( 100, 140, 100, 40 );
    QObject::connect(send_button, SIGNAL ( clicked()), this, SLOT (sendBtnClicked() ) );
    grid->addWidget(send_button, 3, 0, 1, nInner);
    
    for (int i =0; i<nInner; i++) {
        ActionTimedElement* element = 
            new ActionTimedElement(innerActionNames.at(i), innerTimeMargins.at(i));
        grid->addWidget(element, 2, i );
    }
    
    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}

void ActionTimedLayout::sendBtnClicked() {

    std::cout << "TODO, SEND ROS MESSAGE TO TIMED TOPIC" << std::endl;
}
