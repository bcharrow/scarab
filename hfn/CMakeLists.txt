cmake_minimum_required(VERSION 2.8.3)
project(hfn)

find_package(Eigen REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core PATHS /opt/local)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp
             sensor_msgs geometry_msgs nav_msgs tf angles
             actionlib_msgs actionlib message_generation)

add_action_files(DIRECTORY action FILES Move.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

generate_dynamic_reconfigure_options(cfg/HumanFriendlyNavigation.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES playermap
  CATKIN_DEPENDS dynamic_reconfigure roscpp sensor_msgs geometry_msgs
                 nav_msgs tf actionlib_msgs actionlib angles message_runtime
)

include(${CGAL_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_library(playermap src/map.c src/rosmap.cpp)
add_library(hfnlib src/hfn.cpp)
target_link_libraries(hfnlib ${catkin_LIBRARIES})
add_dependencies(hfnlib ${PROJECT_NAME}_gencpp)

if ( CGAL_AUTO_LINK_ENABLED )
  target_link_libraries(hfnlib ${CGAL_3RD_PARTY_LIBRARIES})
else()
  target_link_libraries(hfnlib ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
endif()

add_executable(hfn src/hfn_node.cpp)
target_link_libraries(hfn ${catkin_LIBRARIES} hfnlib playermap)
add_dependencies(hfn ${PROJECT_NAME}_gencfg)
