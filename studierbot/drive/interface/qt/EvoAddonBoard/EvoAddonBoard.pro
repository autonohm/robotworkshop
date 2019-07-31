#-------------------------------------------------
#
# Project created by QtCreator 2019-03-12T00:25:19
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = EvoAddonBoard
TEMPLATE = app

INCLUDEPATH = ../..

SOURCES += main.cpp\
        mainwindow.cpp\
        ../../AddonShieldCAN.cpp\
        ../../SocketCAN.cpp\
        ../../SocketCANObserver.cpp

HEADERS += mainwindow.h \
        ../../AddonShieldCAN.h

FORMS    += mainwindow.ui

QMAKE_CXXFLAGS += -std=c++11
CONFIG += c++11
