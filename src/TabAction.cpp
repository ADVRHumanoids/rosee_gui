#include <rosee_gui/TabAction.h>

TabAction::TabAction(ros::NodeHandle *nh, 
                     std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler,
                     QWidget *parent) : QWidget(parent) {

    this->nh = nh;

    // createActionGroupBox();
    //createJointStateContainer();
    
    //the group box where the singleActionGroupBox will be set.
    //This groupbox will be added with addWidget in TabAction costructor
    containerActionGroupBox = new ContainerActionGroupBox(nh, parent);
    
    jointStateContainer = new JointStateContainer(nh, robotDescriptionHandler, parent);
    
    
    QGridLayout *windowGrid = new QGridLayout;
    windowGrid->addWidget(containerActionGroupBox, 0, 0);
    windowGrid->addLayout(jointStateContainer, 0, 1);
    setLayout(windowGrid);
  
}

void TabAction::createActionGroupBox() {
    
    //the group box where the singleActionGroupBox will be set.
    //This groupbox will be added with addWidget in TabAction costructor
    //containerActionGroupBox = new ContainerActionGroupBox(nh, this);
    
}

void TabAction::createJointStateContainer() {
    
    //jointStateContainer = new JointStateContainer(nh);
    
}




