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

ActionTimedLayout::ActionTimedLayout (std::string actionName, QWidget* parent) : QGroupBox(parent) {
    
    this->setMinimumSize(600,200);
    
    grid = new QGridLayout;
    
    windowLabel = new QLabel (QString::fromStdString(actionName));
    windowLabel->setAlignment(Qt::AlignCenter);
    windowLabel->setStyleSheet("QLabel { font-size : 40px }");
    grid->addWidget(windowLabel, 0, 0, 1, 2); //TODO take 2 (colspan) from number of inners
    
    send_button = new QPushButton ( "SEND", this );
    send_button->setGeometry ( 100, 140, 100, 40 );
    QObject::connect(send_button, SIGNAL ( clicked()), this, SLOT (sendBtnClicked() ) );
    grid->addWidget(send_button, 3, 0, 1, 2); //TODO take 2 (colspan) from number of inners
    
    ActionTimedElement* element1 = new ActionTimedElement("el1");
    grid->addWidget(element1, 2, 0 );
    ActionTimedElement* element2 = new ActionTimedElement("el2");
    grid->addWidget(element2, 2, 1 );
    
    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}
