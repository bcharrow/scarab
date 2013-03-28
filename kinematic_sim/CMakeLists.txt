cmake_minimum_required(VERSION 2.8.3)
project(kinematic_sim)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(kinematic_sim src/kinematic_sim.cc)
target_link_libraries(kinematic_sim ${catkin_LIBRARIES})
