#include <rosee_gui/TimerHandler.h>

TimerHandler::TimerHandler(const rclcpp::Node::SharedPtr node, int msec) : _node ( node ) {
    
    timer = new QTimer(this);
    
    connect(timer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    
    timer->start(msec);
    
}

void TimerHandler::timerSlot() {
    
    // spin ros to deal with pub and sub of all layouts
    rclcpp::spin_some(_node);

}
