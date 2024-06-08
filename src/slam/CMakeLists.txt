cmake_minimum_required(VERSION 3.0.2)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slam
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Define the source files
set(SOURCES
  src/integration.cpp
  src/process_sensor_data.cpp
  src/single_robot_mapping.cpp
)

# Add executable
add_executable(slam ${SOURCES})

# Link libraries
target_link_libraries(slam
  ${catkin_LIBRARIES}
)
