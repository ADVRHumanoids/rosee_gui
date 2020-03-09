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

#ifndef ACTIONTIMEDELEMENT_H
#define ACTIONTIMEDELEMENT_H

#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QProgressBar>

//to set precision of double when converting to string
#include <sstream> 
#include <iomanip>

/**
 * @todo write docs
 */
class ActionTimedElement : public QGroupBox {
    
    Q_OBJECT
public:
    explicit ActionTimedElement(std::string actionName, double before, double after, QWidget* parent=0);
    
    void setProgressBarValue(double);

private: 
    QProgressBar* bar;
};

#endif // ACTIONTIMEDELEMENT_H
