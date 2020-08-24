#include <rosee_gui/SingleActionBoxesGroupBox.h>

SingleActionBoxesGroupBox::SingleActionBoxesGroupBox (ros::NodeHandle* nh, 
                                      rosee_msg::ActionInfo actInfo, QWidget* parent) : 
                                      SingleActionGroupBox(nh, actInfo, parent) {

    if (actInfo.max_selectable > actInfo.selectable_names.size()){
        std::cerr << "[ERROR] max_selectable is " << actInfo.max_selectable << " while you pass only " << actInfo.selectable_names.size()
                  << " names for checkboxes" << std::endl;
        throw "";
    }

    this->maxChecked = actInfo.max_selectable;
    this->actionPrimitiveType = static_cast<ROSEE::ActionPrimitive::Type> (actInfo.actionPrimitive_type);

    this->actualChecked = 0;

    boxes = new QButtonGroup(this);
    boxes->setExclusive(false);

    std::string str = "Check " + std::to_string(maxChecked) + " checkboxe(s) before sending";
    sendBtnTooltip = QString::fromStdString(str);
    /// father members
    //disable until we check the right number of checkboxes
    send_button->setEnabled(false);
    send_button->setToolTip(sendBtnTooltip);

    QGridLayout *boxesLayout = new QGridLayout;
    unsigned int buttonId = 0;
    unsigned int buttonRowColCount = 0; //can be higher that Id because some label may be too long and they do not fit in the second column
    for (auto el : actInfo.selectable_names) {
        QCheckBox* newBox =  new QCheckBox( QString::fromStdString(el));
        newBox->setChecked(false);
        //tooltip on each checkbox so we can read if the label is cut becasue of no space
        newBox->setToolTip(QString::fromStdString(el));
        boxes->addButton ( newBox, buttonId );

        //if names is too long and it is in the second column, go to new line
        if ((buttonRowColCount%2) == 1 && el.size() > 11) {
            buttonRowColCount++;
        }
        
        boxesLayout->addWidget(newBox, buttonRowColCount/2, (buttonRowColCount%2));
        buttonId++;
        buttonRowColCount++;
    }

    QObject::connect( boxes, SIGNAL (buttonClicked(QAbstractButton*)), this,
                      SLOT (clickCheckBoxSlot(QAbstractButton*)) );

    //father grid layout
    grid->addLayout(boxesLayout, 2, 0);

}

SingleActionBoxesGroupBox::SingleActionBoxesGroupBox (ros::NodeHandle* nh, 
                                      rosee_msg::ActionInfo actInfo,
                                      std::map<std::string, std::vector<std::string>> pairedMap,
                                      QWidget* parent) : 
                                      SingleActionGroupBox(nh, actInfo, parent) {

    if (actInfo.max_selectable != 2) {
        std::cerr << "[ERROR] max_selectable is " << actInfo.max_selectable << 
        " this costructor is only for primitive with selectable pairs (for now)" 
        << std::endl;
        
        throw "";
    }
    
    if (actInfo.max_selectable > pairedMap.size()){
        std::cerr << "[ERROR] maxChecked is " << maxChecked << " while you pass only " << pairedMap.size()
                  << " names for checkboxes" << std::endl;
        throw "";
    }
    
    this->maxChecked = actInfo.max_selectable; //which is always 2 for now
    this->actionPrimitiveType = static_cast<ROSEE::ActionPrimitive::Type> (actInfo.actionPrimitive_type);

    this->actualChecked = 0;
    
    this->pairedMap = pairedMap;

    boxes = new QButtonGroup(this);
    boxes->setExclusive(false);

    std::string str = "Check " + std::to_string(maxChecked) + " checkboxe(s) before sending";
    sendBtnTooltip = QString::fromStdString(str);
    /// father members
    //disable until we check the right number of checkboxes
    send_button->setEnabled(false);
    send_button->setToolTip(sendBtnTooltip);

    QGridLayout *boxesLayout = new QGridLayout;
    unsigned int buttonId = 0;
    unsigned int buttonRowColCount = 0; //can be higher that Id because some label may be too long and they do not fit in the second column

    for (auto el : actInfo.selectable_names) {
        QCheckBox* newBox =  new QCheckBox( QString::fromStdString(el));
        newBox->setChecked(false);
        //tooltip on each checkbox so we can read if the label is cut becasue of no space
        newBox->setToolTip(QString::fromStdString(el));
        boxes->addButton ( newBox, buttonId );
        
        //if names is too long and it is in the second column, go to new line
        if ((buttonRowColCount%2) == 1 && el.size() > 11) {
            buttonRowColCount++;
        }

        boxesLayout->addWidget(newBox, buttonRowColCount/2, (buttonRowColCount%2));
        buttonId++;
        buttonRowColCount++;
    }

    QObject::connect( boxes, SIGNAL (buttonClicked(QAbstractButton*)), this,
                      SLOT (clickPairCheckBoxSlot(QAbstractButton*)) );

    //father grid layout
    grid->addLayout(boxesLayout, 2, 0);

}

void SingleActionBoxesGroupBox::sendActionRos () {
    
    rosee_msg::ROSEECommandGoal goal;
    goal.goal_action.seq = rosMsgSeq++ ;
    goal.goal_action.stamp = ros::Time::now();
    goal.goal_action.percentage = getSpinBoxPercentage();
    goal.goal_action.action_name = actionName;
    //singleActionGroupBox can be generic or composed, but it do not change what we put in type
    //because the server will act on them equally
    goal.goal_action.action_type = actionType ;
    goal.goal_action.actionPrimitive_type = actionPrimitiveType ;
    
    //now fill selectable items with the label of all checkboxes checked
    for (auto box : boxes->buttons()) {
        if (box->isChecked()) {
            
            goal.goal_action.selectable_items.push_back(box->text().toUtf8().constData());
        }
    }
    
    action_client->sendGoal (goal, boost::bind(&SingleActionBoxesGroupBox::doneCallback, this, _1, _2),
        boost::bind(&SingleActionBoxesGroupBox::activeCallback, this), boost::bind(&SingleActionBoxesGroupBox::feedbackCallback, this, _1));

}

void SingleActionBoxesGroupBox::clickCheckBoxSlot (QAbstractButton* button) {

    if (button->isChecked()) { //if I clicked to check it...
        actualChecked++;

        if (actualChecked == maxChecked) {

            for (auto box : boxes->buttons()) {
                if (! box->isChecked() ) { //disable only the not checked obviously
                   box->setEnabled(false);
                }
            //enable send button (that is a father member) and remove tooltip
            send_button->setEnabled(true);
            send_button->setToolTip("");
            }
        }

    } else { //If I click to uncheck it
        actualChecked--;
        //for sure now send button must be disables (or it may be already disabled)
        send_button->setEnabled(false);
        send_button->setToolTip(sendBtnTooltip);
        for (auto box : boxes->buttons()) {
            box->setEnabled(true);
        }

    }

}

void SingleActionBoxesGroupBox::clickPairCheckBoxSlot (QAbstractButton* button) {

    if (button->isChecked()) { //if I clicked to check it...
        actualChecked++;
        
    } else { //If I click to uncheck it
        actualChecked--;
    }

    if (actualChecked == 2) {

        for (auto box : boxes->buttons()) {
            if (! box->isChecked() ) { //disable only the not checked obviously
                box->setEnabled(false);
            }
        }
            
        //enable send button (that is a father member) and remove tooltip
        send_button->setEnabled(true);
        send_button->setToolTip("");
            
    } else if (actualChecked == 1) { 
        //we have to disable only the boxes that cant be coupled with the one that is
        // left checked (that is unique because actualChecked == 1)

        disableNotPairedBoxes(boxes->checkedButton()->text().toUtf8().constData());
        send_button->setEnabled(false);
        send_button->setToolTip(sendBtnTooltip);
        
    } else {
        
        for (auto box : boxes->buttons()) {
                box->setEnabled(true);
        }
        
        send_button->setEnabled(false);
        send_button->setToolTip(sendBtnTooltip);
    }
}

void SingleActionBoxesGroupBox::disableNotPairedBoxes( std::string boxName ){
    
     for (auto box : boxes->buttons()) {
                
        if ( box->isChecked() ) { 
            //it is the one checked now...
        } else {
            std::vector<std::string> vect = pairedMap.at(boxName);
            if (std::find(vect.begin(), vect.end(), box->text().toUtf8().constData()) 
                == vect.end()) {
                
                box->setEnabled(false);
            
            } else {
                box->setEnabled(true);

            }
        }
    }
}

void SingleActionBoxesGroupBox::resetAll() {
    
    SingleActionGroupBox::resetAll();
    for (auto box : boxes->buttons()) {
        box->setEnabled(true);
        box->setChecked(false);
    }
    
    actualChecked = 0;
    send_button->setEnabled(false);
}

