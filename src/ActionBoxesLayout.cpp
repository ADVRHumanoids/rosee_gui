#include <rosee_gui/ActionBoxesLayout.h>

ActionBoxesLayout::ActionBoxesLayout (std::string actionName, std::vector<std::string> boxesNames,
                                      unsigned int maxChecked, QWidget* parent) : 
                                      ActionLayout(actionName, parent) {

    if (maxChecked > boxesNames.size()){
        std::cerr << "[ERROR] maxChecked is " << maxChecked << " while you pass only " << boxesNames.size()
                  << " names for checkboxes" << std::endl;
        throw "";
    }

    this->maxChecked = maxChecked;
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
    for (auto el : boxesNames) {
        QCheckBox* newBox =  new QCheckBox( QString::fromStdString(el));
        newBox->setChecked(false);
        //tooltip on each checkbox so we can read if the label is cut becasue of no space
        newBox->setToolTip(QString::fromStdString(el));
        boxes->addButton ( newBox, buttonId );

        boxesLayout->addWidget(newBox, buttonId/2, (buttonId%2));
        buttonId++;
    }

    QObject::connect( boxes, SIGNAL (buttonClicked(QAbstractButton*)), this,
                      SLOT (clickCheckBoxSlot(QAbstractButton*)) );

    //father grid layout
    grid->addLayout(boxesLayout, 2, 0);

}

ActionBoxesLayout::ActionBoxesLayout (std::string actionName,
                                      std::map<std::string, std::vector<std::string>> pairedMap,
                                      QWidget* parent) : 
                                      ActionLayout(actionName, parent) {

    maxChecked = 2; //because we pass that map, this costructor is specific for this kind of action
    if (maxChecked > pairedMap.size()){
        std::cerr << "[ERROR] maxChecked is " << maxChecked << " while you pass only " << pairedMap.size()
                  << " names for checkboxes" << std::endl;
        throw "";
    }

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
    for (auto el : pairedMap) {
        QCheckBox* newBox =  new QCheckBox( QString::fromStdString(el.first));
        newBox->setChecked(false);
        //tooltip on each checkbox so we can read if the label is cut becasue of no space
        newBox->setToolTip(QString::fromStdString(el.first));
        boxes->addButton ( newBox, buttonId );

        boxesLayout->addWidget(newBox, buttonId/2, (buttonId%2));
        buttonId++;
    }

    QObject::connect( boxes, SIGNAL (buttonClicked(QAbstractButton*)), this,
                      SLOT (clickPairCheckBoxSlot(QAbstractButton*)) );

    //father grid layout
    grid->addLayout(boxesLayout, 2, 0);

}

void ActionBoxesLayout::setRosPub(ros::NodeHandle *nh, std::string topicName, MsgType msgType) {

    this->msgType = msgType;
    switch (msgType) {
    case GENERIC: {
        std::cerr << "ERROR, calling setRosPub of a checkboxed layout, Generic should be used only for ActionLayout"
                  << "Please insert a valid type of MsgType as third argument" <<std::endl;
        return;
        break;
    }
    case TRIG: {
        if (maxChecked != 1) {
            std::cerr << "ERROR, calling setRosPub for a Trig action, but maxChecked passed before in the costructor"
                      << "is " << maxChecked << " (should be 1)" <<std::endl;
            return;
        }
        actionPub = nh->advertise<ros_end_effector::EETriggerControl>(topicName, 1);
        break;
    }
    case PINCH : {
        if (maxChecked != 2) {
            std::cerr << "ERROR, calling setRosPub for a Pinch action, but maxChecked passed before in the costructor"
                      << "is " << maxChecked << " (should be 2)" <<std::endl;
            return;
        }
        actionPub = nh->advertise<ros_end_effector::EEPinchControl>(topicName, 1);
        break;
    }
    default : {
        std::cerr << "ERROR " << msgType << " action type not know by GUI" << std::endl;
        return;

    }
    }
}

void ActionBoxesLayout::sendActionRos() {
    switch (msgType) {
    case GENERIC: {
        std::cerr << "ERROR, sending an action from ActionCheckBoxes but the msg type is GENERIC"
                  << std::endl;
        return;

    }
    case TRIG: {
        ros_end_effector::EETriggerControl msg;
        msg.seq = rosMsgSeq++;
        msg.stamp = ros::Time::now();
        msg.percentage = getSpinBoxPercentage();
        msg.finger_trigger = boxes->checkedButton()->text().toUtf8().constData();

        actionPub.publish(msg);

        break;
    }
    case PINCH : {
        ros_end_effector::EEPinchControl msg;
        msg.seq = rosMsgSeq++;
        msg.stamp = ros::Time::now();
        msg.percentage = getSpinBoxPercentage();
        unsigned int nFillFinger = 0;
        for (auto box : boxes->buttons()) {
            //we are sure here only two are set, because we checked in setrospub that maxChecked == 2
            if (box->isChecked()) {
                if (nFillFinger == 0) {
                    msg.finger_pinch_1 = box->text().toUtf8().constData();
                } else if (nFillFinger == 1) {
                    msg.finger_pinch_2 = box->text().toUtf8().constData();
                } else {
                    std::cerr << "[ERROR]" << std::endl; //should never been here for previous checks
                    return;
                }
                nFillFinger++;
            }
        }

        actionPub.publish(msg);

        break;
    }
    default : {
        std::cerr << "ERROR " << msgType << " action type not know by GUI" << std::endl;
        return;

    }
    }


}

void ActionBoxesLayout::clickCheckBoxSlot (QAbstractButton* button) {

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

void ActionBoxesLayout::clickPairCheckBoxSlot (QAbstractButton* button) {

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

void ActionBoxesLayout::disableNotPairedBoxes( std::string boxName ){
    
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


