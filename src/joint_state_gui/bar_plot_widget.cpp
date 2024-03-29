#include "bar_plot_widget.h"
#include "joint_bar_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QComboBox>

void bar_plot_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {


QWidget * LoadUiFile(QWidget * parent)
{
    bar_plot_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/bar_plot_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

}

BarPlotWidget::BarPlotWidget(std::vector<std::string> jnames, QWidget *parent) : QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    setLayout(layout);

    auto bars_layout_left = findChild<QVBoxLayout *>("BarsLayoutLeft");
    auto bars_layout_right = findChild<QVBoxLayout *>("BarsLayoutRight");
    
    bars_layout_left->setSpacing(2);
    bars_layout_right->setSpacing(2);

    for(int i = 0; i < jnames.size(); i++)
    {

        auto wid = new JointBarWidget(QString::fromStdString(jnames[i]),
                                      this);

        if(i < jnames.size() / 2 || jnames.size() < 10)
        {
            bars_layout_left->addWidget(wid);
        }
        else
        {
            bars_layout_right->addWidget(wid);
        }

        wid_map[jnames[i]] = wid;

    }
    
    //remove layout if it is empty to handle better the space
    if (bars_layout_right->count() == 0) {
        bars_layout_right->setParent(nullptr);
        auto line = findChild<QWidget *>("line");
        line->setVisible(false);
    }


    _fieldtype_combobox = findChild<QComboBox *>("SelectFieldComboBox");
    _fieldtype_combobox->addItem("Joint Position");
    _fieldtype_combobox->addItem("Joint Velocity");
    _fieldtype_combobox->addItem("Joint Effort");


}

void BarPlotWidget::setOnJointClicked(std::function<void (std::string)> f)
{
    for(auto pair : wid_map)
    {
//        pair.second->setOnBarDoubleClick(std::bind(f, pair.first));
    }
}

std::string BarPlotWidget::getFieldType() const
{
    return _fieldtype_combobox->currentText().toStdString();
}

std::string BarPlotWidget::getFieldShortType() const
{
    auto type = getFieldType();

    std::map<std::string, std::string> type_to_short_type;
    type_to_short_type["Joint Position" ] = "joint_pos";
    type_to_short_type["Joint Velocity"] = "joint_vel";
    type_to_short_type["Joint Effort" ] = "joint_eff";

    if(type_to_short_type.count(type) > 0)
    {
        return type_to_short_type.at(type);
    }

    return "";
}
