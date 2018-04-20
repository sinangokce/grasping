cmake_minimum_required(VERSION 2.8.3)
project(grasping)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  handtracker
  glove_tekscan_ros_wrapper
# roslaunch  # Disabled due to testing
)

add_message_files(
   FILES
   stop_table.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare executables
add_executable(grasp_type src/grasp_type.cpp src/grasp_type_node.cpp)
target_link_libraries(grasp_type allegro_node ${catkin_LIBRARIES})
add_dependencies(grasp_type ${catkin_EXPORTED_TARGETS})

add_executable(current_listener src/current_listener.cpp src/current_listener_node.cpp)
target_link_libraries(current_listener allegro_node ${catkin_LIBRARIES})

# This is disabled until roslaunch tests can pass required arguments.
# roslaunch_add_file_check(launch/allegro_hand.launch)

#############
## Install ##
#############

install(TARGETS
  grasp_type
  current_listener  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

