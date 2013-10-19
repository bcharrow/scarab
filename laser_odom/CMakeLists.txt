cmake_minimum_required(VERSION 2.8.3)
project(laser_odom)

find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rosbag angles tf sensor_msgs 
  geometry_msgs visualization_msgs)

catkin_package(INCLUDE_DIRS 
   LIBRARIES laser_odom
   CATKIN_DEPENDS roscpp rosbag angles tf sensor_msgs 
  geometry_msgs visualization_msgs)

include_directories(${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS})

add_library(laser_odom src/matcher.cpp)
add_executable(matcher_bag src/matcher_bag.cpp)
target_link_libraries(matcher_bag laser_odom ${catkin_LIBRARIES})

catkin_add_gtest(matcher_test src/matcher_test.cpp)
target_link_libraries(matcher_test laser_odom ${catkin_LIBRARIES})
