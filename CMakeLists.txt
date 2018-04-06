cmake_minimum_required(VERSION 2.8.3)
set(OpenCV_DIR "/home/zhuzunjie/Thirdparty/opencv-3.1.0/build")
project(BGR)

set(CMAKE_CXX_COMPILE "g++")
set(CMAKE_BUILD_TYPE Release)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_library(REALSENSE_LIBS NAMES realsense PATHS /usr/local/lib/x86_64-linux-gnu)

set(DEPENDENCIES ${REALSENSE_LIBS} ${OpenCV_LIBS} ${OPENGL_LIBRARIES})

if(WIN32)
    add_subdirectory(third_party/glfw)
    list(APPEND DEPENDENCIES glfw3)
else()
    # Find glfw header
    find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
        PATHS /usr/X11R6/include
              /usr/include/X11
              /opt/graphics/OpenGL/include
              /opt/graphics/OpenGL/contrib/libglfw
              /usr/local/include
              /usr/include/GL
              /usr/include
/usr/lib/x86_64-linux-gnu
    )
    # Find glfw library
    find_library(GLFW_LIBRARIES NAMES glfw glfw3
            PATHS /usr/lib64
                  /usr/lib
                  /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                  /usr/local/lib64
                  /usr/local/lib
                  /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                  /usr/X11R6/lib
/usr/lib/x86_64-linux-gnu
    )
    list(APPEND DEPENDENCIES m ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES})
    include_directories(${GLFW_INCLUDE_DIR})
endif()

add_executable(dataset main/dataset.cpp)
target_link_libraries(dataset ${DEPENDENCIES}) 

add_executable(device_info main/cpp-enumerate-devices.cpp)
target_link_libraries(device_info ${DEPENDENCIES})