#ifndef ACTIONTRIGLAYOUT_H
#define ACTIONTRIGLAYOUT_H

#include <rosee_gui/ActionBoxesLayout.h>
#include <QWidget>

class ActionTrigLayout : public ActionBoxesLayout
{
    Q_OBJECT
public:
    virtual ~ActionTrigLayout() {}
    ActionTrigLayout(QWidget* parent);
};

#endif // ACTIONTRIGLAYOUT_H
