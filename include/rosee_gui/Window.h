#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QGridLayout>
#include <QGroupBox>


<<<<<<< HEAD
#include <rosee_gui/JointStateContainer.h>
#include <rosee_gui/ContainerActionGroupBox.h>
=======
#include <rosee_msg/ActionsInfo.h>
#include <rosee_msg/SelectablePairInfo.h>
#include <ros_end_effector/Action.h>
>>>>>>> bc173cb... refactor to a single package name convention (with underscores)

class Window : public QWidget
{
    Q_OBJECT
public:
    explicit Window(ros::NodeHandle* nh, QWidget *parent = 0);
    
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

#endif // WINDOW_H
