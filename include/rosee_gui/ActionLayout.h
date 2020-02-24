#ifndef ACTIONLAYOUT_H
#define ACTIONLAYOUT_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QCheckBox>
#include <QGroupBox>

#include <iostream>


class ActionLayout: public QGroupBox
{
    Q_OBJECT
public:
    explicit ActionLayout(std::string actionName, QWidget* parent=0);

public:
    QGridLayout *grid;

protected:
    QPushButton *send_button; //protected so derived class can enable/disable if necessary

private:
    QLabel* windowLabel;
    QSlider *slider_percentage;
    QSpinBox *spinBox_percentage;


signals:

private slots:
    void slotSliderReceive(int value);
    void sendBtnClicked();


};

#endif // ACTIONLAYOUT_H
