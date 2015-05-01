cmake_minimum_required(VERSION 2.8.3)
project(scarab_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs std_msgs geometry_msgs message_generation)

add_action_files(DIRECTORY action FILES Move.action)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_generation std_msgs actionlib_msgs actionlib message_runtime)
