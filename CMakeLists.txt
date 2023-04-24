cmake_minimum_required(VERSION 3.5)

project(SymmetryDemo VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(Open3D_DIR ../Libs/Open3DLib/lib/cmake/Open3D/)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)
find_package(Open3D REQUIRED)
find_package(OpenMP)

include_directories(../Libs/Open3DLib/include/ ../Libs/Open3DLib/include/open3d/3rdparty/ ../Libs)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        constants.h
        OpenGL/abstractopenglwidget.cpp
        OpenGL/abstractopenglwidget.h
        OpenGL/anchorgeometry.cpp
        OpenGL/anchorgeometry.h
        OpenGL/gldisplaymesh.cpp
        OpenGL/gldisplaymesh.h
        OpenGL/gldisplaysymetryvoxels.cpp
        OpenGL/gldisplaysymetryvoxels.h
        OpenGL/gldisplayvoxels.cpp
        OpenGL/gldisplayvoxels.h
        OpenGL/oglobject.cpp
        OpenGL/oglobject.h
        3D/mesh3d.cpp
        3D/mesh3d.h
        3D/voxelgrid.cpp
        3D/voxelgrid.h
        3D/symetry3d.cpp
        3D/symetry3d.h
        mainwindow.ui
)

qt_add_executable(SymmetryDemo
    Resource.qrc
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
)

target_link_libraries(SymmetryDemo PRIVATE Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets OpenMP::OpenMP_CXX Open3D)

set_target_properties(SymmetryDemo PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

qt_finalize_executable(SymmetryDemo)
