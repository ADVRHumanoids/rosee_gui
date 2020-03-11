#include <rosee_gui/ActionLayout.h>

//TODO add as sub-label the type?
ActionLayout::ActionLayout(ros::NodeHandle* nh, rosee_msg::ActionInfo actInfo, QWidget* parent) :
    QGroupBox(parent) {

    this->setMinimumSize(300,200);
    this->actionName = actInfo.action_name;
    this->actionType = static_cast<ActionType> (actInfo.action_type);
    this->rosMsgSeq = 0;

    setRosActionClient(nh, actInfo.ros_action_name);

    grid = new QGridLayout;

    QLabel* titleLabel  = new QLabel (QString::fromStdString(actionName));
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("QLabel { font-size :30px }");
    grid->addWidget(titleLabel, 0, 0);
    
    progressBar = new QProgressBar();
    progressBar->setMaximumHeight(25);
    grid->addWidget(progressBar, 1, 0);

    QHBoxLayout *percentageLayout = new QHBoxLayout;
    slider_percentage = new QSlider(this);
    slider_percentage->setOrientation(Qt::Horizontal);
    slider_percentage->setRange(0, 100);
    slider_percentage->setValue(0);

    spinBox_percentage = new QSpinBox();
    spinBox_percentage->setRange(0,100);
    spinBox_percentage->setSuffix ( " %" );

    //connect slider to spinBox
    QObject::connect(slider_percentage, SIGNAL (valueChanged(int)), spinBox_percentage, SLOT (setValue(int)));
    //connect spinBox to slider
    QObject::connect(spinBox_percentage, SIGNAL (valueChanged(int)), this, SLOT (slotSliderReceive(int)));

    percentageLayout->addWidget ( slider_percentage );
    percentageLayout->addWidget (spinBox_percentage);
    grid->addLayout(percentageLayout, 3, 0);

    send_button = new QPushButton ( "SEND", this );
    send_button->setGeometry ( 100, 140, 100, 40 );
    QObject::connect(send_button, SIGNAL ( clicked()), this, SLOT (sendBtnClicked() ) );
    grid->addWidget(send_button, 4, 0);

    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}

double ActionLayout::getSpinBoxPercentage() {
    return (spinBox_percentage->value()/100.0);
}


void ActionLayout::setRosActionClient ( ros::NodeHandle * nh, std::string rosActionName) {
    
    action_client = 
        std::make_shared <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction>>
        (*nh, rosActionName, false);
        //false because we handle the thread
}

void ActionLayout::sendActionRos () {

    rosee_msg::ROSEECommandGoal goal;
    goal.goal_action.seq = rosMsgSeq++ ;
    goal.goal_action.stamp = ros::Time::now();
    goal.goal_action.percentage = getSpinBoxPercentage();
    goal.goal_action.action_name = actionName;
    //actionLAyout can be generic or composed, but it do not change what we put in type
    //because the server will act on them equally
    goal.goal_action.action_type = ActionType::Generic ;
    goal.goal_action.actionPrimitive_type = PrimitiveType::PrimitiveNone ;
    //goal.goal_action.selectable_items left empty 
    action_client->sendGoal (goal, boost::bind(&ActionLayout::doneCallback, this, _1, _2),
        boost::bind(&ActionLayout::activeCallback, this), boost::bind(&ActionLayout::feedbackCallback, this, _1)) ;

}

void ActionLayout::doneCallback(const actionlib::SimpleClientGoalState& state,
            const rosee_msg::ROSEECommandResultConstPtr& result) {
    
    ROS_INFO_STREAM("[ActionLayout " << actionName << "] Finished in state "<<  state.toString().c_str());
    progressBar->setValue(100);
    
}

void ActionLayout::activeCallback() {
    ROS_INFO_STREAM("[ActionLayout " << actionName << "] Goal just went active");
}

void ActionLayout::feedbackCallback(
    const rosee_msg::ROSEECommandFeedbackConstPtr& feedback) {
    
    ROS_INFO_STREAM("[ActionLayout " << actionName << "] Got Feedback: " << feedback->completation_percentage);
    progressBar->setValue(feedback->completation_percentage);
}


void ActionLayout::slotSliderReceive(int value){

    slider_percentage->setValue(value);

}

void ActionLayout::sendBtnClicked() {

    ROS_INFO_STREAM( "[ActionLayout " << actionName << "] The value is " << getSpinBoxPercentage() );
    ROS_INFO_STREAM( "Sending ROS message..." ) ;
    sendActionRos();
}

