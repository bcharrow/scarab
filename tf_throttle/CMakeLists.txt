cmake_minimum_required(VERSION 2.8.3)
project(tf_throttle)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp tf)
catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp tf
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_throttle src/tf_throttle.cpp)
target_link_libraries(tf_throttle ${catkin_LIBRARIES})
