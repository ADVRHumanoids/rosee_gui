#ifndef JOINT_STATE_WIDGET_H
#define JOINT_STATE_WIDGET_H

#include <QWidget>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QLabel>

class JointStateWidget : public QWidget
{

    Q_OBJECT

public:

    explicit JointStateWidget(QWidget * parent = nullptr);

    void setJointName(QString jname, int jid);

    QDoubleSpinBox * jointPos, * jointVel, * jointEff;
    
    QString getJointName() const { return _jname; }
    void setStatus(std::string status);

signals:

    void plotAdded(QString field);

private:

    QGroupBox * group;
    QString _jname;
    QLabel * _fault;


};

#endif // JOINT_STATE_WIDGET_H
