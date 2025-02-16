cmake_minimum_required(VERSION 3.8)
project(kitti_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(kitti_pub_node src/kitti_pub_node.cpp)
ament_target_dependencies(kitti_pub_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  visualization_msgs 
  OpenCV 
  cv_bridge
)

install(TARGETS
  kitti_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
