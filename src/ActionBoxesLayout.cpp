#include <rosee_gui/ActionBoxesLayout.h>

ActionBoxesLayout::ActionBoxesLayout (std::string actionName, std::vector<std::string> boxesNames, unsigned int maxChecked,
                                       QWidget* parent) : ActionLayout(actionName, parent) {

    if (maxChecked > boxesNames.size()){
        std::cerr << "[ERROR] maxChecked is " << maxChecked << " while you pass only " << boxesNames.size()
                  << " names for checkboxes" << std::endl;
        throw "";
    }

    this->maxChecked = maxChecked;
    this->actualChecked = 0;

    boxes = new QButtonGroup(this);
    boxes->setExclusive(false);

    /// father members
    //disable until we check the right number of checkboxes
    send_button->setEnabled(false);

    QGridLayout *boxesLayout = new QGridLayout;
    unsigned int buttonId = 0;
    for (auto el : boxesNames) {
        QCheckBox* newBox =  new QCheckBox( QString::fromStdString(el));
        newBox->setChecked(false);
        boxes->addButton ( newBox, buttonId );

        boxesLayout->addWidget(newBox, buttonId/2, (buttonId%2));
        buttonId++;
    }

    QObject::connect( boxes, SIGNAL (buttonClicked(QAbstractButton*)), this,
                      SLOT (clickedSlot(QAbstractButton*)) );

    //father grid layout
    grid->addLayout(boxesLayout, 1, 0);

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

void ActionBoxesLayout::clickedSlot (QAbstractButton* button) {

    if (button->isChecked()) {
        actualChecked++;

        if (actualChecked == maxChecked) {

            for (auto box : boxes->buttons()) {
                if (! box->isChecked() ) { //disable only the not checked obviously
                   box->setEnabled(false);
                }
            //enable send button (that is a father member)
            send_button->setEnabled(true);
            }
        }

    } else {
        actualChecked--;
        //for sure now send button must be disables (or it may be already disabled)
        send_button->setEnabled(false);
        for (auto box : boxes->buttons()) {
            box->setEnabled(true);
        }

    }

}

