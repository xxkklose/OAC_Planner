cmake_minimum_required(VERSION 3.14)
project(oac_global_planner LANGUAGES CXX)

# if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "amd64.*|x86_64.*|AMD64.*")
#   enable_language(CUDA)
# endif()

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

catkin_package()
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  roslib
  rviz_visual_tools
  std_msgs
  tf
  grid_map_ros
  livox_ros_driver
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

add_executable(${PROJECT_NAME}_node
    src/oac_global_planning_node.cpp
    src/oac_global_planner.cpp
    src/pf_rrt_planner.cpp
    src/pf_rrt_planner_classes.cpp
    src/pf_rrt_planner_vis.cpp
    )

target_link_libraries(${PROJECT_NAME}_node PUBLIC 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    TBB::tbb
)

# add_executable(local_obs_node
#     src/local_obs.cpp
#     src/planner.cpp
#     src/planner_classes.cpp
#     src/planner_vis.cpp)

# target_link_libraries(local_obs_node 
#       ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES} 
# )