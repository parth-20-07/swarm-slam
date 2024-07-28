cmake_minimum_required(VERSION 3.10)
project(slam)

set(CMAKE_CXX_STANDARD 20)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
set(pybind11_DIR "~/.local/lib/python3.8/site-packages/pybind11/share/cmake/pybind11")
find_package(pybind11 REQUIRED)
find_package(fmt REQUIRED)

if(fmt_FOUND)
  message(STATUS "fmt found: ${fmt_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "fmt not found")
endif()

find_package(Threads REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/ext/eigen
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/ext/matplotlibcpp
  ${pybind11_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
)

add_executable(slam
  src/integration.cpp
  src/process_sensor_data.cpp
  src/single_robot_mapping.cpp
  src/map_merging.cpp
)

target_link_libraries(slam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Python3::Python
  Python3::NumPy
  pybind11::module
  fmt::fmt # Link the fmt library
  ${CMAKE_THREAD_LIBS_INIT} # Link the pthread library
)

install(TARGETS slam
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/random_movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
