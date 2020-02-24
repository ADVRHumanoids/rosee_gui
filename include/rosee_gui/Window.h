#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QGridLayout>

#include <rosee_gui/ActionBoxesLayout.h>
#include <rosee_gui/ActionLayout.h>

class Window : public QWidget
{
    Q_OBJECT
public:
    explicit Window(QWidget *parent = 0);

signals:

public slots:
};

#endif // WINDOW_H
