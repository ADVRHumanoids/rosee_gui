#include <rosee_gui/SingleActionBoxesGroupBox.h>

SingleActionBoxesGroupBox::SingleActionBoxesGroupBox (rclcpp::Node::SharedPtr node, 
                                      rosee_msg::msg::GraspingPrimitiveAggregated graspingPrimitiveAggregated,
                                      std::map<std::string, std::vector<std::string>> pairedMap,
                                      QWidget* parent) : 
                                      SingleActionGroupBox(node, graspingPrimitiveAggregated.action_name, ROSEE::Action::Type::Primitive, parent) {
                                                                                    
    if (graspingPrimitiveAggregated.max_selectable != 2) {
        std::cerr << "[ERROR] max_selectable is " << graspingPrimitiveAggregated.max_selectable << 
        " this costructor is only for primitive with selectable pairs (for now)" 
        << std::endl;
        
        throw "";
    }
    
    if (graspingPrimitiveAggregated.max_selectable > pairedMap.size()){
        std::cerr << "[ERROR] maxChecked is " << maxChecked << " while you pass only " << pairedMap.size()
                  << " names for checkboxes" << std::endl;
        throw "";
    }
    
    this->pairedMap = pairedMap;

    init(graspingPrimitiveAggregated);

    QObject::connect( boxes, SIGNAL (buttonClicked(QAbstractButton*)), this,
                      SLOT (clickPairCheckBoxSlot(QAbstractButton*)) );

}

SingleActionBoxesGroupBox::SingleActionBoxesGroupBox (rclcpp::Node::SharedPtr node, 
                                      rosee_msg::msg::GraspingPrimitiveAggregated graspingPrimitiveAggregated,
                                      QWidget* parent) : 
                                      SingleActionGroupBox(node, graspingPrimitiveAggregated.action_name, ROSEE::Action::Type::Primitive, parent) {

    init(graspingPrimitiveAggregated);

    QObject::connect( boxes, SIGNAL (buttonClicked(QAbstractButton*)), this,
                      SLOT (clickCheckBoxSlot(QAbstractButton*)) );


    
}

bool SingleActionBoxesGroupBox::init( rosee_msg::msg::GraspingPrimitiveAggregated graspingPrimitiveAggregated) {
    
    
    if (graspingPrimitiveAggregated.max_selectable > graspingPrimitiveAggregated.selectable_names.size()){
        std::cerr << "[ERROR] max_selectable is " << graspingPrimitiveAggregated.max_selectable << " while you pass only " << graspingPrimitiveAggregated.selectable_names.size()
                  << " names for checkboxes" << std::endl;
        throw "";
    }

    this->maxChecked = graspingPrimitiveAggregated.max_selectable;
    this->actionPrimitiveType = static_cast<ROSEE::ActionPrimitive::Type> (graspingPrimitiveAggregated.primitive_type);

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
    for (auto el : graspingPrimitiveAggregated.selectable_names) {
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
    
    //father grid layout
    grid->addLayout(boxesLayout, 2, 0);
    
    return true;
}

void SingleActionBoxesGroupBox::sendActionRos () {
    
    using namespace std::placeholders;
    
    auto goal_msg = rosee_msg::action::ROSEECommand::Goal();
    goal_msg.goal_action.seq = rosMsgSeq++ ;
    goal_msg.goal_action.stamp = rclcpp::Clock().now();
    goal_msg.goal_action.percentage = getSpinBoxPercentage();
    goal_msg.goal_action.action_name = actionName;
    //singleActionGroupBox can be generic or composed, but it do not change what we put in type
    //because the server will act on them equally
    goal_msg.goal_action.action_type = actionType ;
    goal_msg.goal_action.action_primitive_type = actionPrimitiveType ;
    
    //now fill selectable items with the label of all checkboxes checked
    for (auto box : boxes->buttons()) {
        if (box->isChecked()) {
            
            goal_msg.goal_action.selectable_items.push_back(box->text().toUtf8().constData());
        }
    }
    
    auto send_goal_options = rclcpp_action::Client<rosee_msg::action::ROSEECommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SingleActionBoxesGroupBox::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SingleActionBoxesGroupBox::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&SingleActionBoxesGroupBox::result_callback, this, _1);
    
    action_client->async_send_goal(goal_msg, send_goal_options);

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

