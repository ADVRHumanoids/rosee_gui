#ifndef ACTIONSPECIFICWINDOW_H
#define ACTIONSPECIFICWINDOW_H

#include <QWidget>
#include <rosee_gui/ActionLayout.h>
#include <QGroupBox>
#include <QCheckBox>

class ActionBoxesLayout : public ActionLayout
{
    Q_OBJECT
public:
    virtual ~ActionBoxesLayout() {}
    ActionBoxesLayout(QWidget* parent=0);
private:
    QGroupBox *groupBox;

protected:
    //virtual void setupCheckBoxes () = 0;
};

#endif // ACTIONSPECIFICWINDOW_H
