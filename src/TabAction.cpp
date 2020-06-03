#include <rosee_gui/TabAction.h>
#include <pluginlib/class_list_macros.h>

namespace rosee_gui {
  
TabAction::TabAction() : rqt_gui_cpp::Plugin(), widget(0) {

    setObjectName("TabAction");
    
    nh = new ros::NodeHandle();
    std::string urdf_file, srdf_file;
    nh->getParam("robot_description", urdf_file);
    nh->getParam("robot_description_semantic", srdf_file);
    
    robotDescriptionHandler = std::make_shared<RobotDescriptionHandler>(urdf_file, srdf_file);

}    
    
TabAction::TabAction(ros::NodeHandle *nh, 
                     std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler,
                     QWidget *parent) : rqt_gui_cpp::Plugin(), widget(parent) {

    setObjectName("ROSEE_GUI");
    
    this->nh = nh;
    
    this->robotDescriptionHandler = robotDescriptionHandler;

    

  
}

void TabAction::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget = new QWidget();
  
          context.addWidget(widget);

    //the group box where the singleActionGroupBox will be set.
    //This groupbox will be added with addWidget in TabAction costructor
    containerActionGroupBox = new ContainerActionGroupBox(nh, widget);

    jointStateContainer = new JointStateContainer(nh, robotDescriptionHandler, widget);


    QGridLayout *windowGrid = new QGridLayout;
    windowGrid->addWidget(containerActionGroupBox, 0, 0);
    windowGrid->addLayout(jointStateContainer, 0, 1);
    widget->setLayout(windowGrid);
    
    // add widget to the user interface
}

void TabAction::shutdownPlugin()
{
    // TODO unregister all publishers here
}

void TabAction::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void TabAction::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

} //namespace

//declare class is deprecated
//PLUGINLIB_DECLARE_CLASS(rosee_gui, TabAction, rosee_gui::TabAction, rqt_gui_cpp::Plugin)

PLUGINLIB_EXPORT_CLASS(rosee_gui::TabAction, rqt_gui_cpp::Plugin)

