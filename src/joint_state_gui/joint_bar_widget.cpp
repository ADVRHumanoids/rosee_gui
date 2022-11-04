#include "joint_bar_widget.h"
#include "circle_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QEvent>
#include <QMouseEvent>
#include <QFrame>

void joint_bar_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {



QWidget * LoadUiFile(QWidget * parent)
{

    joint_bar_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/joint_bar_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

}

JointBarWidget::JointBarWidget(const QString& jname, QWidget *parent) :
    QWidget(parent),
    _blinker(this),
    _state(0)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    layout->setMargin(3);
    setLayout(layout);

    _bar = findChild<QProgressBar *>("ValueBar");
    _bar->installEventFilter(this);
    _jname = findChild<QLabel *>("JointLabel");
    _jname->installEventFilter(this);
    _jname->setText(jname);
    auto font = _jname->font();
    font.setPointSize(12);
    _jname->setFont(font);
    
    //so we can modify the label with (active, passive, mimic) and still return
    //the right joint name
    originalJname = jname;

    _on_double_click = [](){};
}

void JointBarWidget::setRange(double min, double max)
{
    _bar->setRange(min, max);
}

void JointBarWidget::setValue(double x)
{
    _bar->setValue(x);
    //%1 is the placeholder for the arg.
    //arg values: first is value printed (x), 5 is a sort of padding with respect to 
    //the 5 arg ('' by default). 'f' is format, 1 is precision
    _bar->setFormat(QString("%1").arg(x, 5,'f',2));
}

void JointBarWidget::setValue(double xbar, double xtext)
{
    _bar->setValue(xbar);
    _bar->setFormat(QString("%1").arg(xtext, 5,'f',2));
}

void JointBarWidget::setStatus(QString status)
{
//    _status->setText(status);
}

void JointBarWidget::setSafe(bool force)
{
    if(_blinker.blinking()) return;

    if(_state != 0)
    {
        printf("Started blinker..\n");
        _blinker.blink(10);
    }

    _state = 0;

    setColor(Qt::green);
}

void JointBarWidget::setDanger(bool force)
{
    if(_state == 0)
    {
        _blinker.stop();
    }

    setColor(Qt::red);

    _state = 1;

}

void JointBarWidget::setActive()
{
    //not with stylesheet so we do not overwite highlight colors
    //_jname->setStyleSheet("font-weight: bold");
    auto font = _jname->font();
    font.setBold(true);
    _jname->setFont(font);
}

void JointBarWidget::setInactive()
{
    //_jname->setStyleSheet("font-weight: normal");
    auto font = _jname->font();
    font.setBold(false);
    _jname->setFont(font);
}

QString JointBarWidget::getJointName() const
{
    //return _jname->text();
    return originalJname;
}

void JointBarWidget::setColor(Qt::GlobalColor color)
{
    auto frame = findChild<QFrame *>("StatusFrame");
    QPalette pal;
    pal.setColor(QPalette::Background, color);
    frame->setAutoFillBackground(true);
    frame->setPalette(pal);
}

bool JointBarWidget::eventFilter(QObject * obj, QEvent * event)
{
    if (event->type() == QEvent::MouseButtonDblClick)
    {
        QMouseEvent * mouse_event = static_cast<QMouseEvent *>(event);
        if(mouse_event->button() == Qt::MouseButton::LeftButton)
        {
            emit doubleLeftClicked();
        }
        if(mouse_event->button() == Qt::MouseButton::RightButton)
        {
            emit doubleRightClicked();
        }

        return true;
    }
    else
    {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}

Blinker::Blinker(JointBarWidget * parent):
    _blinks(0),
    _state(0),
    _parent(parent)
{
    _timer.setInterval(500);
    _timer.setSingleShot(false);

    connect(&_timer, &QTimer::timeout,
            this, &Blinker::on_timeout);

}

void Blinker::stop()
{
    _blinks = 0;
    _timer.stop();
}

void Blinker::on_timeout()
{
    if(_blinks == 0)
    {
        _parent->setColor(Qt::green);
        _timer.stop();
    }

    printf("Blinking..\n");

    if(_state == 0)
    {
        _parent->setColor(Qt::green);
        _state = 1;
    }
    else {
        _parent->setColor(Qt::red);
        _state = 0;
    }

    _blinks--;

}

void Blinker::blink(int nblinks)
{
    if(_blinks > 0) return;

    _blinks = nblinks;
    _state = 1;
    _timer.start();
}

bool Blinker::blinking() const
{
    return _blinks > 0;
}
