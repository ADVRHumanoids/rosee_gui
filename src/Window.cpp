#include <rosee_gui/Window.h>
#include <rosee_gui/ActionLayout.h>
#include <rosee_gui/ActionBoxesLayout.h>

Window::Window(QWidget *parent) : QWidget(parent) {

    QGridLayout *grid = new QGridLayout;

    QGroupBox* actionLayout = new ActionLayout(this);
    //actionLayout->setTitle("Action");
    grid->addWidget (actionLayout,0,0);


    //QGroupBox* actionBoxesLayout = new ActionBoxesLayout(this) ;
    //actionLayout->setTitle("ActionSpec"); //do not work...
    //grid->addWidget (actionBoxesLayout, 0, 1);



    setLayout(grid);

}

