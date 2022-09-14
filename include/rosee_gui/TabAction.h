#ifndef TABACTION_H
#define TABACTION_H

#include <QWidget>
#include <QGridLayout>
#include <QGroupBox>

#include <rosee_gui/JointStateContainer.h>
#include <rosee_gui/ContainerActionGroupBox.h>
#include <rosee_gui/RobotDescriptionHandler.h>

class TabAction : public QWidget
{
    Q_OBJECT
public:
    explicit TabAction(const rclcpp::Node::SharedPtr node, std::shared_ptr<RobotDescriptionHandler>,  QWidget *parent = 0);
    
private:
    rclcpp::Node::SharedPtr _node;
    ContainerActionGroupBox* containerActionGroupBox;
    
    QGroupBox* actionGroupBox;
    void createActionGroupBox();
    
    JointStateContainer* jointStateContainer;
    void createJointStateContainer();
    



signals:

public slots:
    
    
};

#endif // TABACTION_H
