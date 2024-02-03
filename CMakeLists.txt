cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotinfo_msgs  # Ensure this matches the exact name of your custom message package
  roscpp
  std_srvs
)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Declare a catkin package
catkin_package(
  # CATKIN_DEPENDS is used to specify catkin package dependencies for other packages that depend on this package
  CATKIN_DEPENDS geometry_msgs nav_msgs robotinfo_msgs roscpp std_srvs
  # DEPENDS is used for system dependencies that are found with find_package() and are required by exported libraries
  DEPENDS OpenCV
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(include)

# Add executable
add_executable(${PROJECT_NAME}_node src/robot_gui_node.cpp)

# Link libraries to the executable
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
