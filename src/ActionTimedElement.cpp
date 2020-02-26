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

#include <rosee_gui/ActionTimedElement.h>

ActionTimedElement::ActionTimedElement(std::string actionName, 
                                       std::pair<double, double> margins, QWidget* parent ) : QGroupBox(parent) {
    
    this->setMinimumSize(200,180);
    
    QGridLayout *grid;
    grid = new QGridLayout();

    QLabel* titleLabel;
    titleLabel = new QLabel (QString::fromStdString(actionName));
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("QLabel { font-size :25px }");
    grid->addWidget(titleLabel, 0, 0, 1, 3);
    
    std::ostringstream beforeStream, afterStream;
    beforeStream << std::setprecision(2) << margins.first;
    afterStream << std::setprecision(2) << margins.second;
    std::string timeMarginString = 
        beforeStream.str() + " -- " + actionName + " -- " + afterStream.str();
    QLabel* timeMarginLabel = new QLabel (QString::fromStdString(timeMarginString));
    timeMarginLabel->setAlignment(Qt::AlignCenter);
    timeMarginLabel->setStyleSheet("QLabel { font-size :15px }");
    grid->addWidget(timeMarginLabel, 1, 0, 1, 3);
    
    for (int i = 0; i<3 ; i++) {
        QProgressBar* bar = new QProgressBar();
        bar->setMaximumHeight(20);
        progressBars.push_back(bar);
    }
    if (margins.first == 0.0){
        progressBars.at(0)->setValue(100);
    }
    if (margins.second == 0.0){
        progressBars.at(2)->setValue(100);
    }
    
    grid->addWidget(progressBars[0], 2, 0, 1, 1);
    grid->addWidget(progressBars[1], 3, 0, 1, 3);
    grid->addWidget(progressBars[2], 4, 2, 1, 1);
    
    this->setStyleSheet("QGroupBox { border: 2px dotted black;}");
    this->setLayout(grid);
}


