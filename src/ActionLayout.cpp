#include <rosee_gui/ActionLayout.h>

ActionLayout::ActionLayout(std::string actionName, QWidget* parent) : QGroupBox(parent) {

    this->setMinimumSize(300,200);

    grid = new QGridLayout;

    windowLabel = new QLabel (QString::fromStdString(actionName));
    windowLabel->setAlignment(Qt::AlignCenter);
    windowLabel->setStyleSheet("QLabel { font-size :30px }");
    grid->addWidget(windowLabel, 0, 0, 1, 2);


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
    grid->addLayout(percentageLayout, 2, 0);

    send_button = new QPushButton ( "SEND", this );
    send_button->setGeometry ( 100, 140, 100, 40 );
    QObject::connect(send_button, SIGNAL ( clicked()), this, SLOT (sendBtnClicked() ) );
    grid->addWidget(send_button, 3, 0);

    this->rosMsgSeq = 0;
    this->msgType = GENERIC;

    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);
}

int ActionLayout::getSpinBoxPercentage() {
    return spinBox_percentage->value();
}

void ActionLayout::setRosPub (ros::NodeHandle * nh, std::string topicName, MsgType msgType) {
    actionPub = nh->advertise<rosee_gui::EEGraspControl>(topicName, 1);
}

void ActionLayout::sendActionRos () {

    rosee_gui::EEGraspControl msg;
    msg.seq = rosMsgSeq++;
    msg.stamp = ros::Time::now();
    msg.percentage = spinBox_percentage->value();
    actionPub.publish(msg);

}

void ActionLayout::slotSliderReceive(int value){

    slider_percentage->setValue(value);

}

void ActionLayout::sendBtnClicked() {

    std::cout << "The value is " << spinBox_percentage->value() << std::endl;
    std::cout << "Sending ROS message..." << std::endl;
    sendActionRos();
}

