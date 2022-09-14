#include <rosee_gui/TabAction.h>

TabAction::TabAction(const rclcpp::Node::SharedPtr node, 
                     std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler,
                     QWidget *parent) : QWidget(parent) {

    this->_node = node;

    // createActionGroupBox();
    //createJointStateContainer();
    
    //the group box where the singleActionGroupBox will be set.
    //This groupbox will be added with addWidget in TabAction costructor
    containerActionGroupBox = new ContainerActionGroupBox(_node, parent);
    
    jointStateContainer = new JointStateContainer(_node, robotDescriptionHandler, parent);
    
    
    QGridLayout *windowGrid = new QGridLayout;
    windowGrid->addWidget(containerActionGroupBox, 0, 0);
    windowGrid->addLayout(jointStateContainer, 0, 1);
    setLayout(windowGrid);
  
}

void TabAction::createActionGroupBox() {
    
    //the group box where the singleActionGroupBox will be set.
    //This groupbox will be added with addWidget in TabAction costructor
    //containerActionGroupBox = new ContainerActionGroupBox(_node, this);
    
}

void TabAction::createJointStateContainer() {
    
    //jointStateContainer = new JointStateContainer(_node);
    
}




