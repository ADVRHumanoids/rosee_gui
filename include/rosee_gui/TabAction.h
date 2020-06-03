#ifndef TABACTION_H
#define TABACTION_H

#include <QWidget>
#include <QGridLayout>
#include <QGroupBox>

#include <rosee_gui/JointStateContainer.h>
#include <rosee_gui/ContainerActionGroupBox.h>
#include <rosee_gui/RobotDescriptionHandler.h>

#include <rqt_gui_cpp/plugin.h>

namespace rosee_gui {
    
    
class TabAction : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    TabAction();
    TabAction(ros::NodeHandle* nh, std::shared_ptr<RobotDescriptionHandler>,  QWidget *parent = 0);
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    
private:
    QWidget* widget;
    
    ros::NodeHandle* nh;
    ContainerActionGroupBox* containerActionGroupBox;
    std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler;
    
    QGroupBox* actionGroupBox;
    void createActionGroupBox();
    
    JointStateContainer* jointStateContainer;
    void createJointStateContainer();
    



signals:

public slots:
    
    
};

} // namespace


#endif // TABACTION_H
