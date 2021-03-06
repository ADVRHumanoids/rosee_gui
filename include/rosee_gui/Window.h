#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QGridLayout>

#include <rosee_gui/ActionBoxesLayout.h>
#include <rosee_gui/ActionLayout.h>
#include <rosee_gui/ActionTimedLayout.h>

#include <rosee_msg/ActionsInfo.h>
#include <rosee_msg/SelectablePairInfo.h>

class Window : public QWidget
{
    Q_OBJECT
public:
    explicit Window(ros::NodeHandle* nh, QWidget *parent = 0);
    
private:
    ros::NodeHandle* nh;
    std::vector <rosee_msg::ActionInfo> actionInfoVect;
    
    void getInfoServices() ;
    std::map < std::string, std::vector<std::string> > getPairMap(
        std::string action_name, std::vector<std::string> elements);

signals:

public slots:
};

#endif // WINDOW_H
