#include <rosee_gui/ActionBoxesLayout.h>

ActionBoxesLayout::ActionBoxesLayout (std::string actionName, std::vector<std::string> boxesNames, unsigned int maxChecked,
                                       QWidget* parent) : ActionLayout(actionName, parent) {

    if (maxChecked > boxesNames.size()){
        std::cerr << "[ERROR] maxChecked is " << maxChecked << " while you pass only " << boxesNames.size()
                  << " names for checkboxes" << std::endl;
        throw "";
    }

    this->maxChecked = maxChecked;
    this->actualChecked = 0;

    boxes = new QButtonGroup(this);
    boxes->setExclusive(false);

    // father member... disable until we check the right number of checkboxes
    send_button->setEnabled(false);

    QGridLayout *boxesLayout = new QGridLayout;
    unsigned int buttonId = 0;
    for (auto el : boxesNames) {
        QCheckBox* newBox =  new QCheckBox( QString::fromStdString(el));
        newBox->setChecked(false);
        boxes->addButton ( newBox, buttonId );

        boxesLayout->addWidget(newBox, buttonId/2, (buttonId%2));
        buttonId++;
    }

    QObject::connect( boxes, SIGNAL (buttonClicked(QAbstractButton*)), this,
                      SLOT (clickedSlot(QAbstractButton*)) );

    //father grid layout
    grid->addLayout(boxesLayout, 1, 0);

}

void ActionBoxesLayout::clickedSlot (QAbstractButton* button) {

    if (button->isChecked()) {
        actualChecked++;

        if (actualChecked == maxChecked) {

            for (auto box : boxes->buttons()) {
                if (! box->isChecked() ) { //disable only the not checked obviously
                   box->setEnabled(false);
                }
            //enable send button (that is a father member)
            send_button->setEnabled(true);
            }
        }

    } else {
        actualChecked--;
        //for sure now send button must be disables (or it may be already disabled)
        send_button->setEnabled(false);
        for (auto box : boxes->buttons()) {
            box->setEnabled(true);
        }

    }

}

