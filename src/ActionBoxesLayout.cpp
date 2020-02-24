#include <rosee_gui/ActionBoxesLayout.h>

ActionBoxesLayout::ActionBoxesLayout (QWidget* parent) : ActionLayout(parent) {

    QCheckBox *checkBox1 = new QCheckBox(tr("&Checkbox 1"));
    QCheckBox *checkBox2 = new QCheckBox(tr("C&heckbox 2"));
    checkBox2->setChecked(true);

    QHBoxLayout *vbox = new QHBoxLayout;
    vbox->addWidget(checkBox1);
    vbox->addWidget(checkBox2);

    //father grid layout
    grid->addLayout(vbox, 1, 0);


}
