cmake_minimum_required(VERSION 3.5)

#set project name
set(PROJECT "3DScan")
project(${PROJECT})

#speed up pcl compiling
find_program(CCACHE_FOUND ccache REQUIRED)
if(CCACHE_FOUND)
 set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
 set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)
set (CMAKE_CXX_FLAGS "-DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS")

#find requirements
find_package(PCL 1.8 REQUIRED )
find_package(OpenCV REQUIRED )
find_package(freenect2 REQUIRED )
find_package(VTK REQUIRED )
find_package(Threads )

#link libs
include_directories("/usr/include/libusb-1.0/")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#load src directory
file(GLOB_RECURSE SRCS src/*.cpp)
add_executable(${PROJECT}  ${SRCS})

target_link_libraries(${PROJECT} ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OpenCV_LIBRARIES} ${freenect2_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} )

#libfreenect cmake settings
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

#compilator settings

ADD_DEFINITIONS(
    -std=c++14 # Or -std=c++0x
    # O
)

set_target_properties(${PROJECT} PROPERTIES
        CXX_STANDARD_REQUIRED 14
        CXX_EXTENSIONS ON
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")

