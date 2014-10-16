set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_object_follower)

find_package(catkin REQUIRED COMPONENTS roscpp s8_ip geometry_msgs s8_common_node)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp s8_ip geometry_msgs s8_common_node
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s8_object_follower_node src/object_follower.cpp)
target_link_libraries(s8_object_follower_node ${catkin_LIBRARIES})
