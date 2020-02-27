#include <rosee_gui/Window.h>

Window::Window(ros::NodeHandle *nh, QWidget *parent) : QWidget(parent) {

    this->nh = nh;
    getInfoServices();
    
    QGridLayout *grid = new QGridLayout;
    
    int rowCol = 0;
    
    for (auto actInfo: actionInfoVect) {
        
        if (actInfo.max_selectable == 0) {
            ActionLayout* actionLayout = new ActionLayout(actInfo.action_name, this);
            actionLayout->setRosPub (nh, actInfo.topic_name);
            grid->addWidget (actionLayout, rowCol/4, rowCol%4);
            
        } else if (actInfo.max_selectable == 2) {
            //TODO add a type in the info? or not and continue to use max_selectable as info
            //TODO use another identifier and not name as request?
        
            std::map<std::string, std::vector<std::string>> pairedElementMap = 
                getPairMap(actInfo.action_name, actInfo.selectable_names);
            
            ActionBoxesLayout* actionBoxesLayout;
            if (pairedElementMap.size() != 0) {
                actionBoxesLayout = 
                new ActionBoxesLayout(actInfo.action_name, pairedElementMap, this) ;
                
            } else {
                actionBoxesLayout = 
                new ActionBoxesLayout(actInfo.action_name, actInfo.selectable_names, 
                                      actInfo.max_selectable, this) ; 
            }
            //TODO handle the last argument... PINCH is for message type but can be used for 
            // any action that has 2 names to select
            actionBoxesLayout->setRosPub (nh, actInfo.topic_name, PINCH);
            grid->addWidget (actionBoxesLayout, rowCol/4, rowCol%4);
            
        } 
        //other else for timed action...
        
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
    grid->addWidget(timed, 1, 0, 1, innerActionNames.size());

    setLayout(grid);

}

void Window::getInfoServices() {
    
    ros::service::waitForService("ros_end_effector/ActionsInfo"); //blocking infinite wait, it also print
    
    rosee_msg::ActionsInfo actionsInfo;

    if (ros::service::call ("ros_end_effector/ActionsInfo", actionsInfo)) {
        for (auto actInfo: actionsInfo.response.actionsInfo) {
            actionInfoVect.push_back(actInfo);
        }
    } else {
        //error
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
