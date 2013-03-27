cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB1 libusb-1.0 REQUIRED)
include_directories(${LIBUSB1_INCLUDE_DIRS})

rosbuild_add_library(roboclaw src/RoboClaw.cpp)
target_link_libraries(roboclaw ${LIBUSB1_LDFLAGS})

rosbuild_add_executable(enc_demo src/enc_demo.cpp)
target_link_libraries(enc_demo roboclaw)

rosbuild_add_executable(roboclaw_node src/roboclaw_node.cpp)
target_link_libraries(roboclaw_node roboclaw)

rosbuild_add_executable(roboclaw_restart src/roboclaw_restart.cpp)
target_link_libraries(roboclaw_restart roboclaw)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
