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

add_library(matcher src/matcher.cpp)

add_executable(laser_odom_bag src/laser_odom_bag.cpp)
target_link_libraries(laser_odom_bag matcher ${catkin_LIBRARIES})

add_executable(laser_odom src/laser_odom.cpp)
target_link_libraries(laser_odom matcher ${catkin_LIBRARIES})
