cmake_minimum_required(VERSION 3.14)
project(oac_local_planner LANGUAGES CXX)

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/oac_local_planner_node.cpp
    src/oac_local_planner.cpp
    )

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)