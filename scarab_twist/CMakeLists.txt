cmake_minimum_required(VERSION 2.8.3)
project(scarab_twist)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Curses)

catkin_package(CATKIN_DEPENDS roscpp DEPENDS Curses)

add_executable(joy_to_twist src/joy_to_twist.cpp)
target_link_libraries(joy_to_twist ${catkin_LIBRARIES})

add_executable(key_to_twist src/key_to_twist.cpp)
target_link_libraries(key_to_twist ${catkin_LIBRARIES} ${CURSES_LIBRARIES})
