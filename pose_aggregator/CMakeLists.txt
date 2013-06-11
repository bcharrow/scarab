cmake_minimum_required(VERSION 2.8.3)
project(pose_aggregator)
find_package(catkin REQUIRED COMPONENTS 
                             roscpp std_msgs geometry_msgs message_generation)

add_message_files(FILES PoseStampedNamed.msg PoseStampedNamedArray.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs std_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(pose_aggregator src/pose_aggregator_node.cpp)
target_link_libraries(pose_aggregator ${catkin_LIBRARIES})

