#-------------------------------------------------
#
# Project created by QtCreator 2014-05-18T20:01:31
#
#-------------------------------------------------

QT       -= gui

TARGET = ucimage
TEMPLATE = lib

#CONFIG += staticlib debug
CONFIG += debug

DEFINES += IMAGELIB_LIBRARY


SOURCES += image.cpp \
    gdalexception.cpp \
    imagefuncs.cpp \
    rpc.cpp \
    imagefuncs.h        # keep this here for templated funcs


HEADERS += image.h \
    ucimagelib_global.h \
    gdalexception.h  \
    rpc.h \
    imagefuncs.h \
    rpc_types.h


INCLUDEPATH += ../ucbaselib \
        /usr/include/gdal


LIBS += -L../ucbaselib -lucbase

