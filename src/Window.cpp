#include <rosee_gui/Window.h>

Window::Window(ros::NodeHandle *nh, QWidget *parent) : QWidget(parent) {

    this->nh = nh;
    
    // get (wait) for info from unviersalroseeExecutor about all the action parsed by it
    getInfoServices();
    
    createActionGroupBox();
    createJointStateTable();
    
    QGridLayout *windowGrid = new QGridLayout;
    windowGrid->addWidget(actionGroupBox, 0, 0);
    windowGrid->addWidget(jointStateTable, 0, 1);
    setLayout(windowGrid);
    
  

}

void Window::createJointStateTable() {
    
    jointStateTable = new JointStateTable(0,0, this);
    jointStateTable->setRowCount(1);
    jointStateTable->setColumnCount(1);
    
    QStringList headerLabels;
    headerLabels.append("asd");
    jointStateTable->setHorizontalHeaderLabels(headerLabels);
    
    jointStateTable->setRowCount(2);
    jointStateTable->setColumnCount(2);
    
    
}

void Window::createActionGroupBox() {
    
    //the group box where the actionLayout will be set.
    //This groupbox will be added with addWidget in Window costructor
    actionGroupBox = new QGroupBox;
    
    // the layout that contain all the actions
    actionContainerLayout = new QGridLayout();

    int rowCol = 0;
    
    for (auto actInfo: actionInfoVect) {
            
        switch (actInfo.action_type) {
        
        case ROSEE::Action::Type::Primitive :
        {
            if (actInfo.max_selectable == 2) {
                // get (wait) for service that provide info of which element can be paired
                // so in the gui we disable the not pairable checkboxes if one is checked
                std::map<std::string, std::vector<std::string>> pairedElementMap = 
                    getPairMap(actInfo.action_name, actInfo.selectable_names);
                
                ActionBoxesLayout* actionBoxesLayout;
                if (pairedElementMap.size() != 0) {
                    actionBoxesLayout = 
                    new ActionBoxesLayout(nh, actInfo, pairedElementMap, this) ;
                    
                } else {
                    //version without disabling the not pairable checkboxes
                    actionBoxesLayout = 
                    new ActionBoxesLayout(nh, actInfo, this) ; 
                    
                }
                actionContainerLayout->addWidget (actionBoxesLayout, rowCol/4, rowCol%4);
                
            } else {
                ActionBoxesLayout* actionBoxesLayout;
                actionBoxesLayout = 
                    new ActionBoxesLayout(nh, actInfo, this) ; 
                actionContainerLayout->addWidget (actionBoxesLayout, rowCol/4, rowCol%4);
            }

            break;
        }
        case ROSEE::Action::Type::Generic : // same thing as composed
        case ROSEE::Action::Type::Composed :
        {
            ActionLayout* actionLayout = new ActionLayout(nh, actInfo, this);
            actionContainerLayout->addWidget (actionLayout, rowCol/4, rowCol%4);
            break;
        }
        case ROSEE::Action::Type::Timed : 
        {
            ActionTimedLayout* timed = new ActionTimedLayout(nh, actInfo, this);
            actionContainerLayout->addWidget(timed, rowCol/4, rowCol%4, 1, actInfo.inner_actions.size());

            break;
        }
        case ROSEE::Action::Type::None :
        {
            
            ROS_ERROR_STREAM ("GUI ERROR, type NONE received for action " << actInfo.action_name);
            throw "";
            break;
        }
        default : {
            ROS_ERROR_STREAM ("GUI ERROR, not recognized type " << actInfo.action_type
            << " received for action " << actInfo.action_name);
            throw "";
        }
        } 
        
        rowCol++;
    }

    actionGroupBox->setLayout(actionContainerLayout);
    
}

void Window::getInfoServices() {
    
    ros::service::waitForService("ros_end_effector/ActionsInfo"); //blocking infinite wait, it also print
    
    rosee_msg::ActionsInfo actionsInfo;

    if (ros::service::call ("ros_end_effector/ActionsInfo", actionsInfo)) {
        actionInfoVect = actionsInfo.response.actionsInfo;
    } else {
        ROS_ERROR_STREAM (" ros::service::call FAILED " );
    }
    
}


std::map < std::string, std::vector<std::string> > Window::getPairMap( 
    std::string action_name, std::vector<std::string> elements) {
    
    std::map<std::string, std::vector<std::string>> pairedElementMap;
    
    ROS_INFO_STREAM ("waiting for ros_end_effector/SelectablePairInfo service for 5 seconds...");
    if (! ros::service::waitForService("ros_end_effector/SelectablePairInfo", 5000)) {
        ROS_WARN_STREAM ("ros_end_effector/SelectablePairInfo not found");
        return std::map < std::string, std::vector<std::string> >();
    }
    ROS_INFO_STREAM ("... service found, I will call it");

    rosee_msg::SelectablePairInfo pairInfo;
    pairInfo.request.action_name = action_name;

    for (auto elementName : elements) {
        pairInfo.request.element_name = elementName;
        if (ros::service::call("ros_end_effector/SelectablePairInfo", pairInfo)) {
            
            pairedElementMap.insert(std::make_pair(elementName, pairInfo.response.pair_elements) );
            
        } else {
            ROS_ERROR_STREAM ("ros_end_effector/SelectablePairInfo call failed with " << 
                pairInfo.request.action_name << ", " << pairInfo.request.element_name <<
                " as request");
            return std::map < std::string, std::vector<std::string> >();

        }
    }
    return pairedElementMap;
}
