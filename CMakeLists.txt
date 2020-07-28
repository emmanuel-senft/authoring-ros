cmake_minimum_required(VERSION 2.8.3)
project(authoring)

SET(CMAKE_CXX_FLAGS  "-O3 -fopenmp")

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

find_package(Eigen3 REQUIRED)

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

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

message(STATUS test)
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_executable(pandaJacob src/jacob.cpp)
target_include_directories(pandaJacob PUBLIC adept${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} /home/senft/catkin_ws/devel/include )
target_link_libraries(pandaJacob
   ${catkin_LIBRARIES}
)
#############
## Install ##
#############

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY
    config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY
    rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(DIRECTORY
    meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
install(DIRECTORY
    urdfs/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
)

catkin_install_python(PROGRAMS
    nodes/rviz_viewer.py
    nodes/start_here.py
    nodes/parser
    nodes/planner
    nodes/mover
    nodes/poi
    nodes/simulator
    nodes/static_publisher
    nodes/tf_printer
    nodes/r_ik_wrapper
    nodes/space_mouse
    nodes/virtual_camera_manager
    nodes/pc_publish
    nodes/tracker
    nodes/clipping_task
    nodes/folding_task
    nodes/rviz_manager
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS pandaJacob
  RUNTIME DESTINATION lib/authoring
)

