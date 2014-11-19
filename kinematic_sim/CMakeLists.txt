cmake_minimum_required(VERSION 2.8.3)
project(kinematic_sim)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp tf hfn)

catkin_package(CATKIN_DEPENDS roscpp tf hfn)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_executable(kinematic_sim src/kinematic_sim.cc)
target_link_libraries(kinematic_sim ${catkin_LIBRARIES})
