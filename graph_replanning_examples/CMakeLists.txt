cmake_minimum_required(VERSION 3.0.2)
project(graph_replanning_examples)

add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
graph_replanning
roscpp
)
catkin_package(
  CATKIN_DEPENDS graph_replanning roscpp
  DEPENDS
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(       example_replanner src/example_replanner.cpp)
add_dependencies(     example_replanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_replanner
  ${catkin_LIBRARIES}
)

add_executable(       example_replanner_manager src/example_replanner_manager.cpp)
add_dependencies(     example_replanner_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_replanner_manager
  ${catkin_LIBRARIES}
)
