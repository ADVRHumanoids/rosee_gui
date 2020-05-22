#ifndef TABACTION_H
#define TABACTION_H

#include <QWidget>
#include <QGridLayout>
#include <QGroupBox>

#include <rosee_gui/JointStateContainer.h>
#include <rosee_gui/ContainerActionGroupBox.h>

class TabAction : public QWidget
{
    Q_OBJECT
public:
    explicit TabAction(ros::NodeHandle* nh, QWidget *parent = 0);
    
private:
    ros::NodeHandle* nh;
    ContainerActionGroupBox* containerActionGroupBox;
    
    QGroupBox* actionGroupBox;
    void createActionGroupBox();
    
    JointStateContainer* jointStateContainer;
    void createJointStateContainer();
    



signals:

public slots:
    
    
};

#endif // TABACTION_H
