#include <rosee_gui/TimerHandler.h>

TimerHandler::TimerHandler(int msec) {
    
    timer = new QTimer(this);
    
    connect(timer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    
    timer->start(msec);
    
}

void TimerHandler::timerSlot() {
    
    // spin ros to deal with pub and sub of all layouts
    ros::spinOnce();

}
