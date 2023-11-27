cmake_minimum_required(VERSION 3.0.2)
project(oac_mapping LANGUAGES CXX)

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
  tf_conversions
  grid_map_ros
  grid_map_cv
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
    src/oac_mapping_node.cpp
    src/oac_mapping.cpp
    )

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    TBB::tbb
    -lgmp
)