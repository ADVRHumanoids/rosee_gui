#include <rosee_gui/Window.h>

Window::Window(ros::NodeHandle *nh, QWidget *parent) : QWidget(parent) {

    this->nh = nh;
    getInfoServices();
    
    QGridLayout *grid = new QGridLayout;

    ActionLayout* actionLayout = new ActionLayout("Generic", this);
    actionLayout->setRosPub (nh, "ros_end_effector/grasp");

    grid->addWidget (actionLayout,0,0);

    std::vector <std::string> fingers;
    fingers.push_back("finger_1");
    fingers.push_back("figner_2");
    fingers.push_back("figner_2");
    fingers.push_back("figner_2");
    fingers.push_back("figner_2");
    unsigned int maxChecked = 2;
    ActionBoxesLayout* actionBoxesLayout = new ActionBoxesLayout("Pinch", fingers, maxChecked, this) ;
    actionBoxesLayout->setRosPub (nh, "ros_end_effector/pinch", PINCH);
    grid->addWidget (actionBoxesLayout, 0, 1);

    ActionBoxesLayout* actionBoxesLayout2 = new ActionBoxesLayout("Trig", fingers, 1, this) ;
    actionBoxesLayout2->setRosPub (nh, "ros_end_effector/trig", TRIG);
    grid->addWidget (actionBoxesLayout2, 0, 2);
    
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
    
    //ros::ServiceClient client = nh.serviceClient<> ( " " );
    
}
