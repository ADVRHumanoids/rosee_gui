#include "joint_state_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QComboBox>
#include <QPushButton>

void joint_state_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {


QWidget * LoadUiFile(QWidget * parent)
{
    joint_state_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/joint_state_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

}


JointStateWidget::JointStateWidget(QWidget * parent):
    QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    setLayout(layout);
    setMinimumWidth(400);

    jointEff = findChild<QDoubleSpinBox *>("jointEff");
    jointVel = findChild<QDoubleSpinBox *>("jointVel");
    jointPos = findChild<QDoubleSpinBox *>("jointPos");

    group = findChild<QGroupBox *>("JointStateGroup");

    _fault = findChild<QLabel *>("faulttext");

    jointPos        ->setRange(-1e9, 1e9);
    jointVel        ->setRange(-1e9, 1e9);
    jointEff     ->setRange(-1e9, 1e9);

    auto plot_joint_pos = findChild<QPushButton *>("plotJointPos");
    connect(plot_joint_pos, &QPushButton::released,
            [this](){ emit plotAdded("joint_pos");});

    auto plot_joint_vel = findChild<QPushButton *>("plotJointVel");
    connect(plot_joint_vel, &QPushButton::released,
            [this](){ emit plotAdded("joint_vel");});

    auto plot_joint_eff = findChild<QPushButton *>("plotJointEff");
    connect(plot_joint_eff, &QPushButton::released,
            [this](){ emit plotAdded("joint_effort");});

}

void JointStateWidget::setJointName(QString jname, int jid)
{
    group->setTitle(QString("%1  (ID: %2)").arg(jname).arg(jid));
    _jname = jname;
}

void JointStateWidget::setStatus(std::string status)
{
    _fault->setText(QString::fromStdString(status));
}
