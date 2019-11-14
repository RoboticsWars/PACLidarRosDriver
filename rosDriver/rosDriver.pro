QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

INCLUDEPATH += ../src ../../devel/include ../include
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp \
    ../src/bufferDecode.cpp \
    ../src/ioapi.cpp \
    ../src/publisher.cpp \
    ../src/ssFrameLib.cpp \
    ../src/lidarDriver.cpp

DISTFILES += \
    ../rosdoc.docx \
    ../package.xml \
    ../launch/node_manager.launch \
    ../CMakeLists.txt \
    ../msg/PointStream.msg \
    ../srv/LidarCommand.srv

HEADERS += \
    ../src/bufferDecode.h \
    ../src/cloud_node.h \
    ../src/ioapi.h \
    ../src/ssFrameLib.h \
    ../src/lidarDriver.h \
    ../include/bufferDecode.h \
    ../include/cloud_node.h \
    ../include/ioapi.h \
    ../include/lidarDriver.h \
    ../include/ssFrameLib.h
