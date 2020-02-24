#include <rosee_gui/Window.h>

Window::Window(QWidget *parent) : QWidget(parent) {

    QGridLayout *grid = new QGridLayout;

    QGroupBox* actionLayout = new ActionLayout("Generic", this);
    grid->addWidget (actionLayout,0,0);

    std::vector <std::string> fingers;
    fingers.push_back("finger_1");
    fingers.push_back("figner_2");
    fingers.push_back("figner_2");
    fingers.push_back("figner_2");
    fingers.push_back("figner_2");
    unsigned int maxChecked = 2;
    QGroupBox* actionBoxesLayout = new ActionBoxesLayout("GenericBox", fingers, maxChecked, this) ;
    grid->addWidget (actionBoxesLayout, 0, 1);

    setLayout(grid);

}

