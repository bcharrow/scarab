cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(CGAL QUIET COMPONENTS Core PATHS /opt/local)

if (NOT CGAL_FOUND)
  message(FATAL_ERROR "This program requires the CGAL library, and will not be compiled.")
endif(NOT CGAL_FOUND)

include( ${CGAL_USE_FILE} )

rosbuild_add_library(LaserSimulator src/LaserSimulator.cc)

if ( CGAL_AUTO_LINK_ENABLED )    
  target_link_libraries(LaserSimulator ${CGAL_3RD_PARTY_LIBRARIES} )
else()
  target_link_libraries(LaserSimulator ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
endif()

rosbuild_add_executable(laser_simulator src/laser_simulator.cc)
target_link_libraries(laser_simulator LaserSimulator)
