#include "joint_monitor_widget.h"

struct BIT_FAULT {

    // bits 0..3
    uint16_t  m3_rxpdo_pos_ref:1;   // ?
    uint16_t  m3_rxpdo_vel_ref:1;   // ?
    uint16_t  m3_rxpdo_tor_ref:1;   // ?
    uint16_t  m3_fault_hardware:1;  // ! +
    // bits 4..7
    uint16_t  m3_params_out_of_range:1;         // !
    uint16_t  m3_torque_array_not_loaded:1;     // !
    uint16_t  m3_torque_read_error:1;           // ? ++
    uint16_t  m3_spare:1;
    // bits 8, 9
    /* 20 BAD consecutive sensor readings give error
     * each GOOD or almost good ( green/orange) reading decrement error counter
     * each other conditions on reading increment error counter
     */
    uint16_t  m3_link_enc_error:1;  // ? +
    /* 20 consecutive sensor readings give error
     * see m3_link_enc_error
     */
    uint16_t  m3_defl_enc_error:1;  // ? ++
    /*
    The warning bit is set when the temperature is over 60 and released when decrease under 55 degrees.
    The warning bit is only checked before starting the controller, if is set the controller is not allowed to start.
    If set during run no action is provided, is only a signal that the system is getting warm.
    The error bit is set when the temperature is over 70 and released when decrease under 65 degrees.
    The error bit is always checked, if a temperature error is triggered the system doesn’t start the controller
    and if is running it stop the controller.
    For the motor temperature is the same behavior (the warning and error temperature bit are shared)
    but the temperature limits are different (80 for warning and 90 for error with 5 degrees histeresys)
    */
    // bits 10, 11
    uint16_t  m3_temperature_warning:1; // ?
    uint16_t  m3_temperature_error:1;   // ? +
    // bits 12..15
    /* 200 consecutive sensor readings give error
     * see m3_link_enc_error
     */
    uint16_t  c28_motor_enc_error:1;    // ? +
    uint16_t  c28_v_batt_read_fault:1;  // ?
    uint16_t  c28_enter_sand_box:1; // ?
    uint16_t  c28_spare_1:1;


    std::ostream& dump ( std::ostream& os, const std::string delim ) const {

        if(m3_rxpdo_pos_ref)    { os << "m3_rxpdo_pos_ref" << delim; }
        if(m3_rxpdo_vel_ref)    { os << "m3_rxpdo_vel_ref" << delim; }
        if(m3_rxpdo_tor_ref)    { os << "m3_rxpdo_tor_ref" << delim; }
        if(m3_fault_hardware)   { os << "m3_fault_hardware" << delim; }

        if(m3_params_out_of_range)      { os << "m3_params_out_of_range" << delim; }
        if(m3_torque_array_not_loaded)  { os << "m3_torque_array_not_loaded" << delim; }
        if(m3_torque_read_error) { os << "m3_torque_read_out_of_range" << delim; }

        if(m3_link_enc_error)       { os << "m3_link_enc_error" << delim; }
        if(m3_defl_enc_error)       { os << "m3_defl_enc_error" << delim; }
        if(m3_temperature_warning)  { os << "m3_temperature_warning" << delim; }
        if(m3_temperature_error)    { os << "m3_temperature_error" << delim; }

        if(c28_motor_enc_error)     { os << "c28_motor_enc_error" << delim; }
        if(c28_v_batt_read_fault)   { os << "c28_v_batt_read_fault" << delim; }
        if(c28_enter_sand_box)      { os << "c28_enter_sand_box" << delim; }

        return os;
    }

    void fprint ( FILE *fp ) {
        std::ostringstream oss;
        dump(oss,"\t");
        fprintf ( fp, "%s", oss.str().c_str() );
    }
    int sprint ( char *buff, size_t size ) {
        std::ostringstream oss;
        dump(oss,"\t");
        return snprintf ( buff, size, "%s", oss.str().c_str() );
    }

};

union centAC_fault_t{
    uint16_t all;
    BIT_FAULT bit;
};

JointMonitorWidget::JointMonitorWidget(ros::NodeHandle* nh, 
                                       std::shared_ptr<RobotDescriptionHandler> robotDescriptionHandler, 
                                       QWidget *parent) :
    QWidget(parent),
    _valid_msg_recv(false),
    _widget_started(false)
{
    
    std::string jsTopic;
    nh->param<std::string>("/rosee/joint_states_topic", jsTopic, "/ros_end_effector/joint_states");
    _jstate_sub = nh->subscribe(jsTopic, 10, &JointMonitorWidget::on_jstate_recv, this);
    ROS_INFO_STREAM ( "[2nd Tab] Getting joint pos from '" << jsTopic << "'" );

    this->_robotDescriptionHandler = robotDescriptionHandler;

    _urdf = _robotDescriptionHandler->getUrdfModel();

    while(!_valid_msg_recv)
    {
        ros::spinOnce();
        ROS_INFO_STREAM_ONCE("Waiting for joint states valid message...");
        usleep(1000);
    }
    

    std::string jidmap_str = nh->param<std::string>("robot_description_joint_id_map", "");
    if(!jidmap_str.empty())
    {
        try
        {
            auto jidmap_yaml = YAML::Load(jidmap_str);
            for(auto p: jidmap_yaml["joint_map"])
            {
                _jidmap[p.second.as<std::string>()] = p.first.as<int>();
            }
        }
        catch(std::exception& e)
        {
            fprintf(stderr, "Unable to get joint IDs: %s \n", e.what());
        }
    }


    jstate_wid = new JointStateWidget(this);
    jstate_wid->setJointName(QString::fromStdString(_jnames[0]), 0);

    barplot_wid = new BarPlotWidget(_jnames, this);

    auto on_joint_clicked = [this](std::string jname)
    {
        auto wid = barplot_wid->wid_map.at(jstate_wid->getJointName().toStdString());
        wid->setInactive();
        int jid = _jidmap.count(jname) > 0 ? _jidmap.at(jname) : 0;
        jstate_wid->setJointName(QString::fromStdString(jname), jid);
        wid = barplot_wid->wid_map.at(jname);
        wid->setActive();
    };

    on_joint_clicked(_jnames[0]);

    for(auto p: barplot_wid->wid_map)
    {
        connect(p.second, &JointBarWidget::doubleLeftClicked,
                [p, on_joint_clicked](){ on_joint_clicked(p.first); });

        connect(p.second, &JointBarWidget::doubleRightClicked,
                [p, this]()
        {

            _chart->addSeries(p.second->getJointName() +
                              "/" +
                              QString::fromStdString(barplot_wid->getFieldShortType()));
        });
        
        
        //color the joint names accordingly to actuated type
        ROSEE::JointActuatedType type = robotDescriptionHandler->getActuatedJointsMap().at(p.first);
            
        if (type == ROSEE::JointActuatedType::ACTUATED) {
            
            p.second->findChild<QLabel *>("JointLabel")->setStyleSheet(
                "background-color: rgba(0,255,0,0.5)");

        } else if (type == ROSEE::JointActuatedType::MIMIC) {

            p.second->findChild<QLabel *>("JointLabel")->setStyleSheet(
                "background-color: rgba(255,255,0,0.5)");
            
        } else {
            
            p.second->findChild<QLabel *>("JointLabel")->setStyleSheet(
                "background-color: rgba(255,0,0,0.5)");
        }
    }

    _chart = new ChartWidget;
    _chart->setMinimumSize(640, 400);

    auto layout = new QHBoxLayout(this);
    layout->addWidget(barplot_wid);
    
    auto vlayout = new QVBoxLayout(this);
    vlayout->addWidget(jstate_wid);
    vlayout->addWidget(_chart);

    layout->addLayout(vlayout);
    setLayout(layout);

    connect(jstate_wid, &JointStateWidget::plotAdded,
            [this](QString plot)
            {
                _chart->addSeries(jstate_wid->getJointName() + "/" + plot);
            });


//     _timer = new QTimer(this);
//     _timer->setInterval(40);
//     connect(_timer, &QTimer::timeout,
//             this, &JointMonitorWidget::on_timer_event);
//     _timer->start();

    _widget_started = true;


}

void JointMonitorWidget::on_timer_event()
{
    ros::spinOnce();
}

void JointMonitorWidget::on_jstate_recv(const sensor_msgs::JointStateConstPtr& msg)
{
    if(!_valid_msg_recv)
    {
        _jnames = msg->name;
        _valid_msg_recv = true;
        return;
    }

    if(!_widget_started)
    {
        return;
    }

    static auto t0 = msg->header.stamp;
    auto now = msg->header.stamp;



    for(int i = 0; i < msg->name.size(); i++)
    {

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/joint_pos",
                         (now - t0).toSec(),
                         msg->position[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/joint_vel",
                         (now - t0).toSec(),
                         msg->velocity[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/joint_effort",
                         (now - t0).toSec(),
                         msg->effort[i]);


        if(msg->name[i] == jstate_wid->getJointName().toStdString())
        {
            jstate_wid->jointEff->setValue(msg->effort[i]);
            jstate_wid->jointPos->setValue(msg->position[i]);
            jstate_wid->jointVel->setValue(msg->velocity[i]);
          

            std::string fault_str = "Ok";

            jstate_wid->setStatus(fault_str);

        }

        //if(msg->fault[i] == 0)
        //{
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setSafe();

        //else
        //{
        //   auto wid = barplot_wid->wid_map.at(msg->name[i]);
        //     wid->setDanger();
        //}

            
        if(barplot_wid->getFieldType() == "Joint Effort")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(std::fabs(msg->effort[i]), msg->effort[i]);
            double taumax = _urdf->getJoint(msg->name[i])->limits->effort;
            wid->setRange(0, taumax);

        }
       
        else if(barplot_wid->getFieldType() == "Joint Position")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            //set range want only integer... so I multiply by 100 everything,
            // but the text will be written correct.
            // if we getValue, we will need to divide it by 100
            wid->setValue(msg->position[i]*100, msg->position[i]);
            double qmin = _urdf->getJoint(msg->name[i])->limits->lower;
            double qmax = _urdf->getJoint(msg->name[i])->limits->upper;
            wid->setRange(qmin*100, qmax*100);
        }
        
        else if(barplot_wid->getFieldType() == "Joint Velocity")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(msg->velocity[i]);
            double qdmax = _urdf->getJoint(msg->name[i])->limits->velocity;
            wid->setRange(0, qdmax);
        }
        
    }

}
