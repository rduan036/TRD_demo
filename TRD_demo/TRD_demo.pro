#-------------------------------------------------
#
# Project created by QtCreator 2016-07-17T12:11:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TRD_demo
TEMPLATE = app

INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_contrib
LIBS += -lopencv_legacy
LIBS += -lopencv_flann
LIBS += -lopencv_nonfree

SOURCES += main.cpp\
        mainwindow.cpp \
    TRD_Tracker.cpp \
    homography_decomp.cpp \
    slic.cpp \
    general_function.cpp

HEADERS  += mainwindow.h \
    TRD_Tracker.h \
    homography_decomp.h \
    slic.h \
    general_function.h

FORMS    += mainwindow.ui
