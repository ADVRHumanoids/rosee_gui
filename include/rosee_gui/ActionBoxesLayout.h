#ifndef ACTIONBOXESLAYOUT_H
#define ACTIONBOXESLAYOUT_H

#include <rosee_gui/ActionLayout.h>

#include <QWidget>
#include <QGroupBox>
#include <QCheckBox>
#include <QButtonGroup>

#include <ros_end_effector/EETriggerControl.h>
#include <ros_end_effector/EEPinchControl.h>

class ActionBoxesLayout : public ActionLayout
{
    Q_OBJECT
public:
    explicit ActionBoxesLayout(std::string actionName, std::vector<std::string> boxesNames, unsigned int maxChecked,
                      QWidget* parent=0);

    void setRosPub(ros::NodeHandle * nh, std::string topicName, MsgType msgType) override;

private:
    QButtonGroup* boxes;
    unsigned int maxChecked;
    unsigned int actualChecked;


    void sendActionRos() override;


protected:

private slots:
    void clickedSlot (QAbstractButton* button);

};

#endif // ACTIONBOXESLAYOUT_H
