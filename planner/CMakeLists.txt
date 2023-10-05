cmake_minimum_required(VERSION 3.14)
project(planner LANGUAGES CXX CUDA)

if(CUDA_ENABLED)
  enable_language(CUDA)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

catkin_package()
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rviz_visual_tools
  std_msgs
  tf
)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
message(STATUS "TBB_INCLUDE_DIRS: ${TBB_INCLUDE_DIRS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

add_executable(global_planning_node
    src/global_planning_node.cpp
    src/planner.cpp
    src/planner_classes.cpp
    src/planner_vis.cpp
    src/global_planner.cpp
    )

target_link_libraries(global_planning_node PUBLIC 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    TBB::tbb
)

add_executable(local_obs_node
    src/local_obs.cpp
    src/planner.cpp
    src/planner_classes.cpp
    src/planner_vis.cpp)

target_link_libraries(local_obs_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)