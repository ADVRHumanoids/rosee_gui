#ifndef ACTIONBOXESLAYOUT_H
#define ACTIONBOXESLAYOUT_H

#include <rosee_gui/ActionLayout.h>

#include <QWidget>
#include <QGroupBox>
#include <QCheckBox>
#include <QButtonGroup>

class ActionBoxesLayout : public ActionLayout
{
    Q_OBJECT
public:
    explicit ActionBoxesLayout(std::string actionName, std::vector<std::string> boxesNames, unsigned int maxChecked,
                      QWidget* parent=0);
private:
    QButtonGroup* boxes;
    unsigned int maxChecked;
    unsigned int actualChecked;

protected:

private slots:
    void clickedSlot (QAbstractButton* button);

};

#endif // ACTIONBOXESLAYOUT_H
