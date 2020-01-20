cmake_minimum_required(VERSION 2.8.3)
project(authoring)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        tf 
        cv_bridge 
        image_transport 
        visualization_msgs
        image_geometry 
        ros_markers
        nodelet)

## System dependencies
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
if(2.9.0 VERSION_GREATER ${OpenCV_VERSION})
    message(FATAL "OpenCV3 required!")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_markers
  CATKIN_DEPENDS 
    tf
    nodelet
  DEPENDS OpenCV
  LIBRARIES 
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

#############
## Install ##
#############

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


catkin_install_python(PROGRAMS
    nodes/rviz_viewer.py
    nodes/start_here.py
    nodes/parser
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


