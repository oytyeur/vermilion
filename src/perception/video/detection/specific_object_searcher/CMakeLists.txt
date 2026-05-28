cmake_minimum_required(VERSION 3.8)
project(specific_object_search)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

find_package(unitree_go2 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${unitree_go2_INCLUDE_DIRS}
)

add_executable(specific_object_search_node 
  src/specific_object_search.cpp 
  src/main.cpp
)

target_link_libraries(specific_object_search_node
  ${OpenCV_LIBRARIES}
  ${unitree_go2_LIBRARIES}
)

ament_target_dependencies(specific_object_search_node
  rclcpp
  vision_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  specific_object_search_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()