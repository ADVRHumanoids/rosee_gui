#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QGridLayout>

#include <rosee_gui/ActionBoxesLayout.h>
#include <rosee_gui/ActionLayout.h>
#include <rosee_gui/ActionTimedLayout.h>

#include <rosee_msg/ActionsInfo.h>

class Window : public QWidget
{
    Q_OBJECT
public:
    explicit Window(ros::NodeHandle* nh, QWidget *parent = 0);
    
private:
    ros::NodeHandle* nh;
    void getInfoServices() ;

signals:

public slots:
};

#endif // WINDOW_H
