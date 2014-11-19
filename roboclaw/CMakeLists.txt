cmake_minimum_required(VERSION 2.8.3)
project(roboclaw)
find_package(catkin REQUIRED COMPONENTS 
  message_generation nav_msgs tf dynamic_reconfigure roscpp)

add_message_files(FILES motor_state.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

generate_dynamic_reconfigure_options(cfg/Roboclaw.cfg)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB1 libusb-1.0 REQUIRED)
include_directories(${LIBUSB1_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS 
  nav_msgs tf dynamic_reconfigure roscpp message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_library(roboclaw src/RoboClaw.cpp src/pid.cpp)
target_link_libraries(roboclaw ${LIBUSB1_LDFLAGS} ${roscpp_LIBRARIES})
add_dependencies(roboclaw ${PROJECT_NAME}_generate_messages_cpp)

add_executable(roboclaw_enc_demo src/enc_demo.cpp)
target_link_libraries(roboclaw_enc_demo roboclaw)
set_target_properties(roboclaw_enc_demo PROPERTIES OUTPUT_NAME enc_demo
                      PREFIX "")

add_executable(roboclaw_node src/roboclaw_node.cpp)
target_link_libraries(roboclaw_node roboclaw ${catkin_LIBRARIES})
add_dependencies(roboclaw_node roboclaw_gencfg) 

add_executable(roboclaw_restart src/roboclaw_restart.cpp)
target_link_libraries(roboclaw_restart roboclaw)
