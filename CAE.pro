

CGAL_INCLUDE_PATH = extern/cgal/include
EIGEN_INCLUDE_PATH = extern/eigen

#-------------------------------------------------
#
# Project created by QtCreator 2017-12-25T18:51:04
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets opengl

# Download all external libraries (cgal and eigen) and puts them in
# <CAE-source-dir>/extern/*. Checks if they are already there so this call only
# takes time the first time it's called. Currently only works with header only
# libraries.
system(bash scripts/linux/download_external_libs.sh)

# Automatically downloads all assets that aren't there yet and puts them in
# <build-dir>/assets/*. This call only takes some time the first time it's
# called. The check if a file is already there is very cheap. It's executed
# each time qmake is run.
system(cd $$OUT_PWD && bash $$PWD/scripts/linux/download_assets.sh)

TARGET = CAE
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
#DEFINES += QT_NO_DEBUG_OUTPUT

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# this is always required if some features of cgal are used
# when using CMAKE a similar line is NOT required due to better implementation
DEFINES += CGAL_EIGEN3_ENABLED
DEFINES += CGAL_DONT_OVERRIDE_CMAKE_FLAGS
DEFINES += CGAL_DISABLE_ROUNDING_MATH_CHECK=ON
DEFINES += "BOOST_PARAMETER_MAX_ARITY=12"


CONFIG += console c++14

CONFIG(debug, debug|release){

message("debug")
QMAKE_CXXFLAGS += -O0
QMAKE_CXXFLAGS -= -O1
QMAKE_CXXFLAGS -= -O2
QMAKE_CXXFLAGS -= -O3
CONFIG += debug

} else {

message("release")
#QMAKE_CXXFLAGS += -O0
#QMAKE_CXXFLAGS += -O1
#QMAKE_CXXFLAGS += -O2
QMAKE_CXXFLAGS += -O3
#QMAKE_CXXFLAGS += -Ofast
#QMAKE_CXXFLAGS += -O3
#QMAKE_CXXFLAGS += -march=native
QMAKE_CXXFLAGS += -DNDEBUG

CONFIG += force_debug_info

}

SOURCES = $$files(src/*.cpp, true)
HEADERS = $$files(src/*.h, true) \
    src/data_structures/OrientedAABB.h

FORMS += \
    src/mainwindow.ui \
    src/modules/geometry_info/ui/GeometryInfoUIForm.ui \
    src/modules/interpolator/ui/InterpolatorUIForm.ui \
    src/modules/mesh_converter/ui/MeshConverterUIForm.ui \
    src/simulation/ui/SimulationUIWidget.ui \
    src/modules/demo_loader/ui/DemoLoaderUIForm.ui

# boost
LIBS += -lboost_system
LIBS += -lboost_thread
LIBS += -lboost_filesystem

unix:LIBS        += -lgmp
unix:LIBS        += -lmpfr # not really needed for me, but added since gmp had to be added too
#QMAKE_CXXFLAGS   += -frounding-math# -O3
#QMAKE_CXXFLAGS   += -fp-model strict
INCLUDEPATH += src/
INCLUDEPATH += "$${EIGEN_INCLUDE_PATH}"
INCLUDEPATH += "$${CGAL_INCLUDE_PATH}"

# OpenMp
QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp

LIBS += -lglut -lGLU -lGLEW -lboost_system

SUBDIRS += CAE.pro

#assets.path += $${OUT_PWD}/assets
#assets.files += assets/*
#INSTALLS += assets
