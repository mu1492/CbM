#########################
# cmake, CUDA, and OS requirements
#########################
cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

if( NOT UNIX )
    message( FATAL_ERROR "This project is for Linux systems only." )
endif()

#########################
# Project config
#########################
set(USE_CUDA ON)

set(USE_ACCEL_ADXL355 ON)
set(USE_ACCEL_ADXL357 OFF)

# check if one accelerometer was enabled
if( NOT USE_ACCEL_ADXL355 AND NOT USE_ACCEL_ADXL357 )
    message( FATAL_ERROR "No accelerometer was enabled." )
elseif( USE_ACCEL_ADXL355 AND USE_ACCEL_ADXL357 )
    message( FATAL_ERROR "Multiple accelerometers are not supported." )
endif()

#########################
# I2C clock
#########################
# enable High speed only if 3400 kHz is supported
set(I2C_HIGH_SPEED_SUPPORTED OFF)

# enable Fast+ speed only if 1000 kHz is supported
set(I2C_FAST_PLUS_SUPPORTED ON)

# enable Fast speed only if 400 kHz is supported
set(I2C_FAST_SUPPORTED ON)

if( I2C_HIGH_SPEED_SUPPORTED AND ((NOT I2C_FAST_PLUS_SUPPORTED) OR (NOT I2C_FAST_SUPPORTED)))
    message( FATAL_ERROR "I2C_HIGH_SPEED_SUPPORTED requires lower speeds to be enabled." )
endif()

if( I2C_FAST_PLUS_SUPPORTED AND (NOT I2C_FAST_SUPPORTED) )
    message( FATAL_ERROR "I2C_FAST_PLUS_SUPPORTED requires lower speeds to be enabled." )
endif()

#########################
# Build defines
#########################
if(USE_CUDA)
    add_definitions( -DBUILD_CUDA=1 )
endif()

if(USE_ACCEL_ADXL355)
    add_definitions( -DBUILD_ADXL355=1 )
elseif(USE_ACCEL_ADXL357)
    add_definitions( -DBUILD_ADXL357=1 )
endif()

if(I2C_HIGH_SPEED_SUPPORTED)
    add_definitions( -DBUILD_I2C_HIGH_SPEED=1 )
endif()

if(I2C_FAST_PLUS_SUPPORTED)
    add_definitions( -DBUILD_I2C_FAST_PLUS=1 )
endif()

if(I2C_FAST_SUPPORTED)
    add_definitions( -DBUILD_I2C_FAST=1 )
endif()

#########################
# CUDA flags
#########################
if(USE_CUDA)
    include_directories( "./CUDA" )
    set(CMAKE_CUDA_COMPILER         "/usr/local/cuda/bin/nvcc")

    ##################################################
    # The build includes support for the following NVidia GPU CCs (Compute Capabilities):
    # - 7.2 (e.g. Jetson Xavier NX)                                   
    # - 8.7 (e.g. Jetson Orin Nano)
    # Add one more line in the next section for each new target architecture XX, e.g.
    #    -gencode=arch=compute_XX,code=sm_XX
    ##################################################
    set(CMAKE_CUDA_FLAGS            "-gencode=arch=compute_72,code=sm_72 \
                                     -gencode=arch=compute_87,code=sm_87 \
                                     -gencode=arch=compute_87,code=compute_87" )

    set(CMAKE_CUDA_FLAGS_DEBUG      "-g -G")
    set(CMAKE_CUDA_FLAGS_MINSIZEREL "-Os -DNDEBUG")
    set(CMAKE_CUDA_FLAGS_RELEASE    "-O2 -DNDEBUG")
endif()

#########################
# Project definitions
#########################
if(NOT USE_CUDA)
    project(SpectralViewer LANGUAGES CXX)
else()
    project(SpectralViewerCuda LANGUAGES CXX CUDA)
endif()

#########################
# Build flags
#########################
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenGL_GL_PREFERENCE LEGACY)

#########################
# Packages
#########################
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets OpenGL OpenGLExtensions)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets OpenGL OpenGLExtensions)
find_package(OpenGL REQUIRED)

if(USE_CUDA)
    find_package(CUDA REQUIRED)
endif()

#########################
# Include paths
#########################
include_directories(SYSTEM)

if(USE_ACCEL_ADXL355)
    include_directories("../Drivers/Adxl355/" "../Drivers/Adxl355Adxl357Common/")
elseif(USE_ACCEL_ADXL357)
    include_directories("../Drivers/Adxl357/" "../Drivers/Adxl355Adxl357Common/")
endif()

if(USE_CUDA)
    include_directories(${CUDA_INCLUDE_DIRS})
endif()

#########################
# Project files
#########################
set(PRJ_SOURCES_LIST  )
list(APPEND PRJ_SOURCES_LIST
        main.cpp
        SpectralViewer.cpp
        SpectralViewer.h
        DaqThread.cpp
        DaqThread.h
        FrequencyAnalysis.cpp
        FrequencyAnalysis.h
        WindowFunction.cpp
        WindowFunction.h
        FftThread.cpp
        FftThread.h
        SrsThread.cpp
        SrsThread.h
        VibrationMonitoringSettings.cpp
        VibrationMonitoringSettings.h
        VibrationHandler.cpp
        VibrationHandler.h
        Numeric.cpp
        Numeric.h
        CbmCanvas.cpp
        CbmCanvas.h
        Plot2d.cpp
        Plot2d.h
        Plot2dCanvas.cpp
        Plot2dCanvas.h
        Plot3d.cpp
        Plot3d.h
        Plot3dCanvas.cpp
        Plot3dCanvas.h
        SpectralViewer.ui
        Accelerometer.ui
        FrequencyAnalysis.ui
        VibrationMonitoringSettings.ui
        Plot2dOptions.ui
        Plot3dOptions.ui
        Plot2d.ui
        Plot3d.ui
        About.ui
        SpectralViewer.qrc
        ../Drivers/Adxl355Adxl357Common/Adxl355Adxl357Common.cpp
        ../Drivers/Adxl355Adxl357Common/Adxl355Adxl357Common.h
)

if(USE_ACCEL_ADXL355)
    list(APPEND PRJ_SOURCES_LIST
        ../Drivers/Adxl355/Adxl355.cpp
        ../Drivers/Adxl355/Adxl355.h
        )
elseif(USE_ACCEL_ADXL357)
    list(APPEND PRJ_SOURCES_LIST
        ../Drivers/Adxl357/Adxl357.cpp
        ../Drivers/Adxl357/Adxl357.h
        )
endif()

if(USE_CUDA)
    list(APPEND PRJ_SOURCES_LIST
        cuMl.cu
        cuMl.hpp
        )
endif()

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    if(NOT USE_CUDA)
        qt_add_executable(SpectralViewer MANUAL_FINALIZATION ${PRJ_SOURCES_LIST})
    else()
        qt_add_executable(SpectralViewerCuda MANUAL_FINALIZATION ${PRJ_SOURCES_LIST})
    endif()
else()
    if(NOT USE_CUDA)
        add_executable(SpectralViewer ${PRJ_SOURCES_LIST})
    else()
        add_executable(SpectralViewerCuda ${PRJ_SOURCES_LIST})
    endif()
endif()

if(USE_CUDA)
    set_target_properties(SpectralViewerCuda PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

#########################
# Linker and libraries
#########################
if(NOT USE_CUDA)
    target_link_libraries(SpectralViewer PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
                                                 Qt${QT_VERSION_MAJOR}::OpenGL
                                                 Qt${QT_VERSION_MAJOR}::OpenGLExtensions
                                                 ${OPENGL_LIBRARIES})
else()
    target_compile_options(SpectralViewerCuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
                           -std=c++11
                           -m64
                           -O2
                           -Xcompiler
                           >)

    set(CUDA_LIBRARIES "libcuda.so")
    target_link_libraries(SpectralViewerCuda PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
                                                     Qt${QT_VERSION_MAJOR}::OpenGL
                                                     Qt${QT_VERSION_MAJOR}::OpenGLExtensions
                                                     ${OPENGL_LIBRARIES}
                                                     ${CUDA_LIBRARIES}
                                                     -lcufft
                                                     -lcudla)
endif()

if(QT_VERSION_MAJOR EQUAL 6)
    if(NOT USE_CUDA)
        qt_finalize_executable(SpectralViewer)
    else()
        qt_finalize_executable(SpectralViewerCuda)
    endif()
endif()
