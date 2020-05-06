#include <rosee_gui/Window.h>

Window::Window(ros::NodeHandle *nh, QWidget *parent) : QWidget(parent) {

    this->nh = nh;

    createActionGroupBox();
    createJointStateContainer();
    
    QGridLayout *windowGrid = new QGridLayout;
    windowGrid->addWidget(containerActionGroupBox, 0, 0);
    windowGrid->addLayout(jointStateContainer, 0, 1);
    setLayout(windowGrid);
    
  

}


void Window::createJointStateContainer() {
    
    jointStateContainer = new JointStateContainer(nh);
    
}

void Window::createActionGroupBox() {
    
    //the group box where the singleActionGroupBox will be set.
    //This groupbox will be added with addWidget in Window costructor
    containerActionGroupBox = new ContainerActionGroupBox(nh, this);
    
}


