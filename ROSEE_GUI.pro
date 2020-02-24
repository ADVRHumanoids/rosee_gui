TEMPLATE = app
TARGET = rosee_gui

QT = core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

SOURCES += \
    src/main.cpp \
    src/Window.cpp \
    src/ActionLayout.cpp \
    src/ActionBoxesLayout.cpp \
    src/ActionTrigLayout.cpp

HEADERS += \
    include/rosee_gui/Window.h \
    include/rosee_gui/ActionLayout.h \
    include/rosee_gui/ActionBoxesLayout.h \
    include/rosee_gui/ActionTrigLayout.h
