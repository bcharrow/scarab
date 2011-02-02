cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

exec_program("rospack" ARGS "find acroname" OUTPUT_VARIABLE ACRONAME_PKG_PATH)

rosbuild_add_executable(example src/example.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aMotion.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aStemCore.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aUtil.c)
target_link_libraries(example m aIO aStem)

rosbuild_add_executable(set_baud_rate src/set_baud_rate.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aMotion.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aStemCore.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aUtil.c)
target_link_libraries(set_baud_rate m aIO aStem)

rosbuild_add_executable(check_baud_rate src/check_baud_rate.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aMotion.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aStemCore.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aUtil.c)
target_link_libraries(check_baud_rate m aIO aStem)


rosbuild_add_executable(acroname_moto_node src/acroname_moto_node.cc
  src/AcronameMotor.cc
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aMotion.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aStemCore.c 
  ${ACRONAME_PKG_PATH}/brainstem/aCommon/aUtil.c)
target_link_libraries(acroname_moto_node m aIO aStem)