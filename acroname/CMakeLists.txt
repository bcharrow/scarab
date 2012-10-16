cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# rosbuild_init()

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/brainstem make
  RESULT_VARIABLE _make_failed)

if(_make_failed)
  message(FATAL_ERROR "Build of acroname brainstem failed")
endif(_make_failed)
