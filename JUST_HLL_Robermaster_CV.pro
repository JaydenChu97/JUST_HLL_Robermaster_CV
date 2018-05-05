#-------------------------------------------------
#
# Project created by QtCreator 2017-11-24T21:43:33
#
#-------------------------------------------------
QT += serialport
QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = JUST_HLL_Robermaster_CV
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += F:/opencv3.3.1_contrib_mingw32/include\
               F:/opencv3.3.1_contrib_mingw32/include/opencv\
               F:/opencv3.3.1_contrib_mingw32/include/opencv2


LIBS += F:/opencv3.3.1_contrib_mingw32/lib/libopencv_*.dll.a

FORMS += \
    code/mainwindow.ui

HEADERS += \
    code/armour_detector.h \
    code/armour_tracker.h \
    code/mainwindow.h \
    code/tool.h \
    code/serial.h \
    code/camera.h \
    code/video.h \
    code/common.h \
    code/image.h \
    code/control.h

SOURCES += \
    code/armour_detector.cpp \
    code/armour_tracker.cpp \
    code/mainwindow.cpp \
    code/tool.cpp \
    code/main.cpp \
    code/serial.cpp \
    code/camera.cpp \
    code/video.cpp \
    code/image.cpp \
    code/control.cpp

DISTFILES += \
    statics/params.xml

DEFINES += DEBUG

QMAKE_CXXFLAGS += -Wno-sign-compare
