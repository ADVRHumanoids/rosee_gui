#include <rosee_gui/SingleActionGroupBox.h>

//TODO add as sub-label the type?
SingleActionGroupBox::SingleActionGroupBox(const rclcpp::Node::SharedPtr node, std::string actionName, 
                                           ROSEE::Action::Type actionType,
                                           QWidget* parent) : QGroupBox(parent) {
                                               
    _node = node;

    this->setMinimumSize(120,140);
    this->setMaximumSize(400,400);
    this->actionName = actionName;
    this->actionType = actionType;
    this->rosMsgSeq = 0;

    setRosActionClient();

    grid = new QGridLayout;

    QLabel* titleLabel  = new QLabel (QString::fromStdString(actionName));
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("QLabel { font-size :25px }");
    titleLabel->setMaximumHeight(40);
    titleLabel->setToolTip(QString::fromStdString(actionName));
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
    send_button->setMinimumSize(50,20);
    QObject::connect(send_button, SIGNAL ( clicked()), this, SLOT (sendBtnClicked() ) );
    grid->addWidget(send_button, 4, 0);

    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}

double SingleActionGroupBox::getSpinBoxPercentage() {
    return (spinBox_percentage->value()/100.0);
}


void SingleActionGroupBox::setRosActionClient () {
    
    action_client = rclcpp_action::create_client<rosee_msg::action::ROSEECommand>(
            _node,
            "/action_command");

        //(*nh, "/ros_end_effector/action_command", false);
        //false because we handle the thread
}

void SingleActionGroupBox::sendActionRos () {

    using namespace std::placeholders;
    
    auto goal_msg = rosee_msg::action::ROSEECommand::Goal();
    goal_msg.goal_action.seq = rosMsgSeq++ ;
    goal_msg.goal_action.stamp = rclcpp::Clock().now();
    goal_msg.goal_action.percentage = getSpinBoxPercentage();
    goal_msg.goal_action.action_name = actionName;
    goal_msg.goal_action.action_type = actionType ;
    //action layout is never for primitives, primitives always use singleActionBoxesGroupBox
    goal_msg.goal_action.action_primitive_type = ROSEE::ActionPrimitive::None ;
    //goal_msg.goal_action.selectable_items left empty 
    
    auto send_goal_options = rclcpp_action::Client<rosee_msg::action::ROSEECommand>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SingleActionGroupBox::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SingleActionGroupBox::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&SingleActionGroupBox::result_callback, this, _1);
    
    action_client->async_send_goal(goal_msg, send_goal_options);

}



void SingleActionGroupBox::goal_response_callback(std::shared_future<GoalHandleGraspingActionROS::SharedPtr> future)
{
    RCLCPP_INFO_STREAM(_node->get_logger(), "[SingleActionGroupBox " << actionName << "] Goal just went active");
}

void SingleActionGroupBox::feedback_callback(
    GoalHandleGraspingActionROS::SharedPtr,
    const std::shared_ptr<const rosee_msg::action::ROSEECommand::Feedback> feedback) {
    
    RCLCPP_INFO_STREAM(_node->get_logger(), "[SingleActionGroupBox " << actionName << "] Got Feedback: " << feedback->completation_percentage);
    progressBar->setValue(feedback->completation_percentage);
}

void SingleActionGroupBox::result_callback(
    const GoalHandleGraspingActionROS::WrappedResult & result) {
    
    RCLCPP_INFO_STREAM(_node->get_logger(), "[SingleActionGroupBox " << actionName << "] Finished");
    progressBar->setValue(100);
    
}


void SingleActionGroupBox::slotSliderReceive(int value){

    slider_percentage->setValue(value);

}

void SingleActionGroupBox::sendBtnClicked() {

    RCLCPP_INFO_STREAM(_node->get_logger(), "[SingleActionGroupBox " << actionName << "] The value is " << getSpinBoxPercentage() );
    RCLCPP_INFO_STREAM(_node->get_logger(), "Sending ROS message..." ) ;
    sendActionRos();
}

void SingleActionGroupBox::resetAll() {
    progressBar->setValue(0);
    slider_percentage->setValue(0);
}
