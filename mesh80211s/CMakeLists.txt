cmake_minimum_required(VERSION 2.8.3)
project(mesh80211s)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs)

add_message_files(FILES MeshStation.msg MeshStations.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBNL1 libnl-1 REQUIRED)
include_directories(${LIBNL1_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/mesh80211s.cc src/station.c)
target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${LIBNL1_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
