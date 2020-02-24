#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QGridLayout>

class Window : public QWidget
{
    Q_OBJECT
public:
    virtual ~Window() {}
    explicit Window(QWidget *parent = 0);

signals:

public slots:
};

#endif // WINDOW_H
