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
                                       double before, double after, QWidget* parent ) : QGroupBox(parent) {
    
    this->setMinimumSize(130,130);
    this->setMaximumSize(200,200);
    
    QGridLayout *grid;
    grid = new QGridLayout();

    QLabel* titleLabel;
    titleLabel = new QLabel (QString::fromStdString(actionName));
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("QLabel { font-size :20px }");
    grid->addWidget(titleLabel, 0, 0);
    
    std::ostringstream beforeStream, afterStream;
    beforeStream << std::setprecision(2) << before;
    afterStream << std::setprecision(2) << after;
    std::string timeMarginString = 
        beforeStream.str() + " -- " + actionName + " -- " + afterStream.str();
    QLabel* timeMarginLabel = new QLabel (QString::fromStdString(timeMarginString));
    timeMarginLabel->setAlignment(Qt::AlignCenter);
    timeMarginLabel->setStyleSheet("QLabel { font-size :15px }");
    grid->addWidget(timeMarginLabel, 1, 0);
    
    bar = new QProgressBar();
    bar->setMaximumHeight(20);

    grid->addWidget(bar, 2, 0);
    
    this->setStyleSheet("QGroupBox { border: 2px dotted black;}");
    this->setLayout(grid);
}

void ActionTimedElement::setProgressBarValue(double value) {
    bar->setValue(value);
}

void ActionTimedElement::resetAll() {
    bar->setValue(0);
}

