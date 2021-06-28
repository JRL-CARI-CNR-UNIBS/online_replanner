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

add_executable(       test_replanner src/test/test_replanner.cpp)
add_dependencies(     test_replanner ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_replanner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       test_collision_check src/test/test_collision_check.cpp)
add_dependencies(     test_collision_check ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_collision_check
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       test_replanner_manager src/test/test_replanner_manager.cpp)
add_dependencies(     test_replanner_manager ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_replanner_manager
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(       test_projection src/test/test_projection.cpp)
add_dependencies(     test_projection ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_projection
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       test_path_validity src/test/test_path_validity.cpp)
add_dependencies(     test_path_validity ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_path_validity
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(       test_parallel_checker src/test/test_parallel_checker.cpp)
add_dependencies(     test_parallel_checker ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_parallel_checker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(       test_replanner_manager_real src/test/test_replanner_manager_real.cpp)
add_dependencies(     test_replanner_manager_real ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_replanner_manager_real
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(       test_new_start src/test/test_new_start.cpp)
add_dependencies(     test_new_start ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_new_start
  ${PROJECT_NAME}
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
