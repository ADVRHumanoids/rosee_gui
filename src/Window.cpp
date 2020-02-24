#include <rosee_gui/Window.h>

Window::Window(ros::NodeHandle *nh, QWidget *parent) : QWidget(parent) {

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

    setLayout(grid);

}

