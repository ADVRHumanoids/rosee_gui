#include <rosee_gui/ActionLayout.h>

ActionLayout::ActionLayout(std::string actionName, QWidget* parent) : QGroupBox(parent) {

    this->setMinimumSize(300,200);

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
    spinBox_percentage->setSuffix ( " %");

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

    this->rosMsgSeq = 0;
    this->msgType = GENERIC;

    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}

double ActionLayout::getSpinBoxPercentage() {
    return (spinBox_percentage->value()/100.0);
}

void ActionLayout::setRosPub (ros::NodeHandle * nh, std::string topicName, MsgType msgType) {
    actionPub = nh->advertise<ros_end_effector::EEGraspControl>(topicName, 1);
}

void ActionLayout::setRosActionClient ( ros::NodeHandle * nh, std::string rosActionName) {
    
    action_client = 
        std::make_shared <actionlib::SimpleActionClient <rosee_msg::ROSEECommandAction>>
        (*nh, rosActionName);
        //TODO send Action ROS -> sendGoal....
}

void ActionLayout::sendActionRos () {

    ros_end_effector::EEGraspControl msg;
    msg.seq = rosMsgSeq++;
    msg.stamp = ros::Time::now();
    msg.percentage = getSpinBoxPercentage();
    actionPub.publish(msg);

}

void ActionLayout::slotSliderReceive(int value){

    slider_percentage->setValue(value);

}

void ActionLayout::sendBtnClicked() {

    std::cout << "The value is " << getSpinBoxPercentage() << std::endl;
    std::cout << "Sending ROS message..." << std::endl;
    progressBar->setValue(1);
    sendActionRos();
}

