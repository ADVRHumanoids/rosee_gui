#include <rosee_gui/ActionLayout.h>

ActionLayout::ActionLayout(QWidget* parent) : QGroupBox(parent) {

    grid = new QGridLayout;
    setFixedSize(300,200);

    windowLabel = new QLabel ("ActionName");
    windowLabel->setAlignment(Qt::AlignCenter);
    //windowLabel->setGeometry ( 100, 10, 100, 40 );
    grid->addWidget(windowLabel, 0, 0, 1, 2);


    QHBoxLayout *percentageLayout = new QHBoxLayout;
    slider_percentage = new QSlider(this);
    slider_percentage->setOrientation(Qt::Horizontal);
    slider_percentage->setRange(0, 100);
    slider_percentage->setValue(0);

    spinBox_percentage = new QSpinBox();
    spinBox_percentage->setRange(0,100);
    spinBox_percentage->setSuffix ( " %");

    //connect slider to spinBox
    QObject::connect(slider_percentage, SIGNAL (valueChanged(int)), spinBox_percentage, SLOT (setValue(int)));
    //connect spinBox to slider
    QObject::connect(spinBox_percentage, SIGNAL (valueChanged(int)), this, SLOT (slotSliderReceive(int)));

    percentageLayout->addWidget ( slider_percentage );
    percentageLayout->addWidget (spinBox_percentage);
    grid->addLayout(percentageLayout, 2, 0);

    send_button = new QPushButton ( "SEND", this );
    send_button->setGeometry ( 100, 140, 100, 40 );
    QObject::connect(send_button, SIGNAL ( clicked()), this, SLOT (sendBtnClicked() ) );
    grid->addWidget(send_button, 3, 0);


    this->setStyleSheet("QGroupBox { border: 2px solid black;}");
    this->setLayout(grid);

}

void ActionLayout::slotSliderReceive(int value){

    slider_percentage->setValue(value);

}

void ActionLayout::sendBtnClicked() {

    std::cout << "The value is " << spinBox_percentage->value() << std::endl;
}

