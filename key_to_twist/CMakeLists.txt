cmake_minimum_required(VERSION 2.8.3)
project(key_to_twist)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Curses REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp DEPENDS Curses)

add_executable(key_to_twist src/key_to_twist.cpp)
target_link_libraries(key_to_twist ${catkin_LIBRARIES} ${CURSES_LIBRARIES})
