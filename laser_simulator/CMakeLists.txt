cmake_minimum_required(VERSION 2.8.3)
project(laser_simulator)
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs rosbag sensor_msgs tf
  message_generation)

add_message_files(FILES ScanPair.msg)
generate_messages(DEPENDENCIES sensor_msgs nav_msgs)

find_package(Armadillo REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core PATHS /opt/local)
find_package(LAPACK REQUIRED)
include( ${CGAL_USE_FILE} )

catkin_package(CATKIN_DEPENDS roscpp nav_msgs rosbag sensor_msgs tf)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(laser_simlib src/LaserSimulator.cc)

if ( CGAL_AUTO_LINK_ENABLED )    
  target_link_libraries(laser_simlib ${CGAL_3RD_PARTY_LIBRARIES} )
else()
  target_link_libraries(laser_simlib ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
endif()

target_link_libraries(laser_simlib 
  ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(laser_simulator src/laser_simulator.cc)
target_link_libraries(laser_simulator laser_simlib)

add_executable(make_scan_pairs src/make_scan_pairs.cpp)
target_link_libraries(make_scan_pairs 
  laser_simlib ${rosbag_LIBRARIES} ${tf_LIBRARIES})
