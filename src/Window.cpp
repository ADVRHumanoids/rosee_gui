#include <rosee_gui/Window.h>

Window::Window(ros::NodeHandle *nh, QWidget *parent) : QWidget(parent) {

    this->nh = nh;
    
    // get (wait) for info from unviersalroseeExecutor about all the action parsed by it
    getInfoServices();
    
    QGridLayout *grid = new QGridLayout;
    
    int rowCol = 0;
    
    for (auto actInfo: actionInfoVect) {
            
        //TODO add a type in the info? or not and continue to use max_selectable as discriminant
        //TODO use another identifier and not name as request?
        
        if (actInfo.max_selectable == 0) {

            ActionLayout* actionLayout = new ActionLayout(actInfo.action_name, this);
            actionLayout->setRosPub (nh, actInfo.topic_name);
            grid->addWidget (actionLayout, rowCol/4, rowCol%4);
            
        } else if (actInfo.max_selectable == 1) {
            
            ActionBoxesLayout* actionBoxesLayout;
            actionBoxesLayout = 
                new ActionBoxesLayout(actInfo.action_name, actInfo.selectable_names, 
                                      actInfo.max_selectable, this) ; 
                                      
            actionBoxesLayout->setRosPub (nh, actInfo.topic_name, MsgType::TRIG);
            grid->addWidget (actionBoxesLayout, rowCol/4, rowCol%4);
            
        } else if (actInfo.max_selectable == 2) {
        
            // get (wait) for service that provide info of which element can be paired
            // so in the gui we disable the not pairable checkboxes if one is checked
            std::map<std::string, std::vector<std::string>> pairedElementMap = 
                getPairMap(actInfo.action_name, actInfo.selectable_names);
            
            ActionBoxesLayout* actionBoxesLayout;
            if (pairedElementMap.size() != 0) {
                actionBoxesLayout = 
                new ActionBoxesLayout(actInfo.action_name, pairedElementMap, this) ;
                
            } else {
                //version without disabling the not pairable checkboxes
                actionBoxesLayout = 
                new ActionBoxesLayout(actInfo.action_name, actInfo.selectable_names, 
                                      actInfo.max_selectable, this) ; 
            }
            //TODO handle the last argument... PINCH is for message type but can be used for 
            // any action that has 2 names to select
            actionBoxesLayout->setRosPub (nh, actInfo.topic_name, MsgType::PINCH);
            grid->addWidget (actionBoxesLayout, rowCol/4, rowCol%4);
            
        } else {
            //TODO still need the message to send 3 or more elements along with jointPos
            ROS_WARN_STREAM ( "gui for a max_selectable == " << 
                (unsigned)actInfo.max_selectable << " action still not implemented ");
            rowCol--;
        } 
        
        //TODO other else for timed action...
        
        rowCol++;
    }

    /*
    ActionLayout* actionLayout = new ActionLayout("Generic", this);
    actionLayout->setRosPub (nh, "ros_end_effector/grasp");

    grid->addWidget (actionLayout,0,0);

    std::vector <std::string> fingers;
    fingers.push_back("thumb");
    fingers.push_back("index");
    fingers.push_back("middle");
    fingers.push_back("ring");
    fingers.push_back("pinky");
    
    unsigned int maxChecked = 2;
    ActionBoxesLayout* actionBoxesLayout = new ActionBoxesLayout("Pinch", fingers, maxChecked, this) ;
    actionBoxesLayout->setRosPub (nh, "ros_end_effector/pinch", PINCH);
    grid->addWidget (actionBoxesLayout, 0, 1);
    */
    
    std::vector<std::string> innerActionNames;
    innerActionNames.push_back("spread_fing");
    innerActionNames.push_back("grasp");
    std::vector<std::pair<double,double>> innerTimeMargins;
    innerTimeMargins.push_back(std::make_pair(0, 0.2) );
    innerTimeMargins.push_back(std::make_pair(0.5, 0) );
    ActionTimedLayout* timed = new ActionTimedLayout("wide_grasp", innerActionNames, 
                                                     innerTimeMargins, this);
    grid->addWidget(timed, rowCol/4, rowCol%4, 1, innerActionNames.size());

    setLayout(grid);

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
    
    ros::service::waitForService("ros_end_effector/SelectablePairInfo");
    rosee_msg::SelectablePairInfo pairInfo;
    pairInfo.request.action_name = action_name;

    for (auto elementName : elements) {
        pairInfo.request.element_name = elementName;
        if (ros::service::call("ros_end_effector/SelectablePairInfo", pairInfo)) {
            
            pairedElementMap.insert(std::make_pair(elementName, pairInfo.response.pair_elements) );
            
        } else {
            //error
        }
    }
    return pairedElementMap;
}
