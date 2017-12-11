QT += core gui widgets

CONFIG += c++11

INCLUDEPATH += /opt/ros/indigo/include

SOURCES += main.cpp \
    mainwindow.cpp \
    nodeqt.cpp \
    odometrymap.cpp \
    astar.cpp \
    pathcontroller.cpp

HEADERS += \
    mainwindow.h \
    nodeqt.h \
    odometrymap.h \
    astar.h \
    common.h \
    pathcontroller.h
