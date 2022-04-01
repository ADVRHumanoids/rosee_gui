#ifndef SINGLEACTIONBOXESGROUPBOX_H
#define SINGLEACTIONBOXESGROUPBOX_H

#include <rosee_gui/SingleActionGroupBox.h>

#include <QWidget>
#include <QGroupBox>
#include <QCheckBox>
#include <QButtonGroup>
#include <ros/ros.h>

#include <rosee_msg/GraspingAction.h> //msg
#include <rosee_msg/GraspingPrimitiveAggregated.h> //msg
#include <end_effector/GraspingActions/ActionPrimitive.h>

class SingleActionBoxesGroupBox : public SingleActionGroupBox
{
    Q_OBJECT
public:
    explicit SingleActionBoxesGroupBox(ros::NodeHandle* nh, rosee_msg::GraspingPrimitiveAggregated,
                               std::map<std::string, std::vector<std::string>> pairedMap, QWidget* parent=0);
    explicit SingleActionBoxesGroupBox(ros::NodeHandle* nh, rosee_msg::GraspingPrimitiveAggregated, 
                                       QWidget* parent=0);
        
    virtual void resetAll() override;

private:
    QButtonGroup* boxes;
    unsigned int maxChecked;
    ROSEE::ActionPrimitive::Type actionPrimitiveType;
    unsigned int actualChecked;
    QString sendBtnTooltip;
    
    std::map<std::string, std::vector<std::string>> pairedMap; //if not passed in costructor, it will remain empty

    //used by the two costructors for primitives
    bool init(rosee_msg::GraspingPrimitiveAggregated);
    
    void sendActionRos() override;
    
    void disableNotPairedBoxes( std::string boxName );



protected:

private slots:
    void clickCheckBoxSlot (QAbstractButton* button);
    void clickPairCheckBoxSlot (QAbstractButton* button); //for version with pairedmap
    


};

#endif // SINGLEACTIONBOXESGROUPBOX_H
