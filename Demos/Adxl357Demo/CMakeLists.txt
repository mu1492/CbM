cmake_minimum_required(VERSION 3.5)

project(Adxl357Demo LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenGL_GL_PREFERENCE LEGACY)

find_package(Qt5 COMPONENTS Widgets OpenGL OpenGLExtensions REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(SYSTEM "../../Drivers/Adxl357/" "../../Drivers/Adxl355Adxl357Common/")

add_executable(Adxl357Demo
    main.cpp
    Adxl357Demo.cpp
    Adxl357Demo.h
    Adxl357Demo.ui
    GlWidget.cpp
    GlWidget.h
    ../../Drivers/Adxl357/Adxl357.cpp
    ../../Drivers/Adxl357/Adxl357.h
    ../../Drivers/Adxl355Adxl357Common/Adxl355Adxl357Common.cpp
    ../../Drivers/Adxl355Adxl357Common/Adxl355Adxl357Common.h
)

target_link_libraries(Adxl357Demo PRIVATE Qt5::Widgets Qt5::OpenGL Qt5::OpenGLExtensions ${OPENGL_LIBRARIES})
