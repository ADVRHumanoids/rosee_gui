#ifndef ACTIONBOXESLAYOUT_H
#define ACTIONBOXESLAYOUT_H

#include <rosee_gui/ActionLayout.h>

#include <QWidget>
#include <QGroupBox>
#include <QCheckBox>
#include <QButtonGroup>
#include <ros/ros.h>

#include <rosee_msg/ActionInfo.h> //msg
#include <ros_end_effector/ActionPrimitive.h>

class ActionBoxesLayout : public ActionLayout
{
    Q_OBJECT
public:
    explicit ActionBoxesLayout(ros::NodeHandle* nh, rosee_msg::ActionInfo,  QWidget* parent=0);
    explicit ActionBoxesLayout(ros::NodeHandle* nh, rosee_msg::ActionInfo,
                               std::map<std::string, std::vector<std::string>> pairedMap, QWidget* parent=0);

private:
    QButtonGroup* boxes;
    unsigned int maxChecked;
    ROSEE::ActionPrimitive::Type actionPrimitiveType;
    unsigned int actualChecked;
    QString sendBtnTooltip;
    
    std::map<std::string, std::vector<std::string>> pairedMap; //if not passed in costructor, it will remain empty

    void sendActionRos() override;
    
    void disableNotPairedBoxes( std::string boxName );



protected:

private slots:
    void clickCheckBoxSlot (QAbstractButton* button);
    void clickPairCheckBoxSlot (QAbstractButton* button); //for version with pairedmap
    


};

#endif // ACTIONBOXESLAYOUT_H
