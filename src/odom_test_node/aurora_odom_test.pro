QT += core gui widgets

CONFIG += c++11

INCLUDEPATH += /opt/ros/indigo/include

SOURCES += main.cpp \
    mainwindow.cpp \
    nodeqt.cpp \
    odometrymap.cpp

HEADERS += \
    mainwindow.h \
    nodeqt.h \
    odometrymap.h
