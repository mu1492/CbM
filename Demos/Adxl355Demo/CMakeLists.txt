cmake_minimum_required(VERSION 3.5)

project(Adxl355Demo LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets OpenGL OpenGLExtensions REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(SYSTEM "../../Drivers/Adxl355/")

add_executable(Adxl355Demo
    main.cpp
    Adxl355Demo.cpp
    Adxl355Demo.h
    Adxl355Demo.ui
    GlWidget.cpp
    GlWidget.h
    ../../Drivers/Adxl355/Adxl355.cpp
    ../../Drivers/Adxl355/Adxl355.h
)

target_link_libraries(Adxl355Demo PRIVATE Qt5::Widgets Qt5::OpenGL Qt5::OpenGLExtensions ${OPENGL_LIBRARIES})
