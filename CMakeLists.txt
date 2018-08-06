cmake_minimum_required(VERSION 2.8)

project(3DScan)

find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
find_package(OpenCV REQUIRED )
find_package(freenect2 REQUIRED )
find_package( Threads )

include_directories("/usr/include/libusb-1.0/")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test3 main.cpp kinect.cpp)
target_link_libraries(test3 ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OpenCV_LIBRARIES} ${freenect2_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} )

FIND_LIBRARY(freenect2_LIBRARY freenect2
    PATHS @CMAKE_INSTALL_PREFIX@/lib
    NO_DEFAULT_PATH
)
SET(freenect2_LIBRARIES ${freenect2_LIBRARY} @LIBFREENECT2_THREADING_LIBRARIES@)
FIND_PATH(freenect2_INCLUDE_DIR @PROJECT_NAME@/libfreenect2.hpp
    PATHS @CMAKE_INSTALL_PREFIX@/include
    NO_DEFAULT_PATH
)
SET(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR})

#${freenect2_LIBS}

#SET(COMPILE_FLAGS "-std=c++11")

