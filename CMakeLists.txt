cmake_minimum_required(VERSION 2.8.3)
project(robot_info)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotinfo_msgs
  nav_2d_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp robotinfo_msgs nav_2d_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/robot_info_class.cpp
  src/agv_robot_info_class.cpp
)
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(robot_info_node src/robot_info_main.cpp)
add_executable(agv_robot_info_node src/agv_robot_info_main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_info_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)

target_link_libraries(agv_robot_info_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)

## Install executables
install(TARGETS robot_info_node agv_robot_info_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install libraries
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
