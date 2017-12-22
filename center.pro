#-------------------------------------------------
#
# Project created by QtCreator 2017-09-29T07:07:09
#
#-------------------------------------------------

QT       += core gui
QT       += network
QT       += opengl
QT       += svg
QT       += printsupport
QT       += concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = center
TEMPLATE = app

DEFINES += GSL_DLL
#INCLUDEPATH += /media/rome/LENOVO/e/code/RGruopControlCenter/center_0.3/3rd_lib/include
#LIBS += -L/media/rome/LENOVO/e/code/RGruopControlCenter/center_0.3/3rd_lib -lgsl
#LIBS += -L/media/rome/LENOVO/e/code/RGruopControlCenter/center_0.3/3rd_lib -lgslcblas
#LIBS += -L/media/rome/LENOVO/e/code/RGruopControlCenter/center_0.3/3rd_lib -lqwt
INCLUDEPATH += E:\myWork\RGruopControlCenter\center_0.3\3rd_lib\include
LIBS += -LE:\myWork\RGruopControlCenter\center_0.3\3rd_lib\ -lgsl
LIBS += -LE:\myWork\RGruopControlCenter\center_0.3\3rd_lib\ -lgslcblas
#LIBS += -LE:\myWork\RGruopControlCenter\center_0.3\3rd_lib\ -lqwt
LIBS += -LE:\myWork\RGruopControlCenter\center_0.3\3rd_lib\ -lqwtd

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    GNC/controller.cpp \
    io/rudpread.cpp \
    io/rudpsend.cpp \
    plane/planemodel.cpp \
    plane/frame.cpp \
    plane/propulsion.cpp \
    plane/earthandatmosphere.cpp \
    data/datamanager.cpp \
    data/logSystem.cpp \
    3rd_lib/GeomagnetismLibrary.c \
    GUI/plotdialog.cpp \    
    GNC/guidance.cpp \
    GNC/navigation.cpp

HEADERS += \
        mainwindow.h \
    GNC/controller.h \
    io/net_fdm.h \
    io/rudpread.h \
    io/rudpsend.h \
    plane/planemodel.h \
    plane/frame.h \
    plane/propulsion.h \
    plane/earthandatmosphere.h \
    data/datamanager.h \
    data/logSystem.h \
    config/config.h \
    GUI/plotdialog.h \
    GNC/guidance.h \
    GNC/navigation.h

FORMS += \
        mainwindow.ui \
    GUI/plotdialog.ui

include( qwtfunctions.pri )

#RESOURCES     = application.qrc
#DISTFILES +=     config/egm96.txt
