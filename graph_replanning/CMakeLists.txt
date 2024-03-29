cmake_minimum_required(VERSION 3.0.2)
project(graph_replanning)
add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
graph_core
roscpp
object_loader_msgs
moveit_planning_helper
geometry_msgs
configuration_msgs
sound_play
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_replanning
  CATKIN_DEPENDS  graph_core roscpp object_loader_msgs moveit_planning_helper geometry_msgs configuration_msgs sound_play
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_library(${PROJECT_NAME}
src/${PROJECT_NAME}/trajectory.cpp
src/${PROJECT_NAME}/moveit_utils.cpp
src/${PROJECT_NAME}/replanner.cpp
src/${PROJECT_NAME}/replanner_manager.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
