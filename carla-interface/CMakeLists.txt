cmake_minimum_required(VERSION 3.1.0)
project(carla_shell_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(COMMON_DEPS
  carla_msgs  
  rospy
  std_msgs
  nodelet
  swri_nodelet
)

find_package(catkin REQUIRED COMPONENTS
  ${COMMON_DEPS})

catkin_package(
  LIBRARIES carla_shell_bridge
  CATKIN_DEPENDS
    ${COMMON_DEPS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/nodelets/carla_vehicle_interface.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

set_target_properties(
  ${PROJECT_NAME} 
  PROPERTIES COMPILE_FLAGS "-std=c++14 -DPCL_NO_PRECOMPILE"
)

swri_nodelet_add_node(carla_simulation_vehicle_interface_node ${PROJECT_NAME} CarlaSimulationVehicleInterface)
target_link_libraries(carla_simulation_vehicle_interface_node ${PROJECT_NAME})
set_target_properties(carla_simulation_vehicle_interface_node PROPERTIES COMPILE_FLAGS "-std=c++14 -DPCL_NO_PRECOMPILE")
add_dependencies(carla_simulation_vehicle_interface_node ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  carla_simulation_vehicle_interface_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS 
  scripts/traffic.sh 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/generate_traffic.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})