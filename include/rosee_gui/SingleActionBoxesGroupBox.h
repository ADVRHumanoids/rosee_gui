#ifndef SINGLEACTIONBOXESGROUPBOX_H
#define SINGLEACTIONBOXESGROUPBOX_H

#include <rosee_gui/SingleActionGroupBox.h>

#include <QWidget>
#include <QGroupBox>
#include <QCheckBox>
#include <QButtonGroup>
#include <rclcpp/rclcpp.hpp>

#include <rosee_msg/msg/grasping_action.hpp> //msg
#include <rosee_msg/msg/grasping_primitive_aggregated.hpp> //msg
#include "ActionPrimitive.h"

class SingleActionBoxesGroupBox : public SingleActionGroupBox
{
    Q_OBJECT
public:
    explicit SingleActionBoxesGroupBox(const rclcpp::Node::SharedPtr node, rosee_msg::msg::GraspingPrimitiveAggregated,
                               std::map<std::string, std::vector<std::string>> pairedMap, QWidget* parent=0);
    explicit SingleActionBoxesGroupBox(const rclcpp::Node::SharedPtr node, rosee_msg::msg::GraspingPrimitiveAggregated, 
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
    bool init(rosee_msg::msg::GraspingPrimitiveAggregated);
    
    void sendActionRos() override;
    
    void disableNotPairedBoxes( std::string boxName );



protected:

private slots:
    void clickCheckBoxSlot (QAbstractButton* button);
    void clickPairCheckBoxSlot (QAbstractButton* button); //for version with pairedmap
    


};

#endif // SINGLEACTIONBOXESGROUPBOX_H
