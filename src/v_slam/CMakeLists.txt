cmake_minimum_required(VERSION 3.8)
project(v_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Enforce use of C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# find package 
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(glog REQUIRED)


# include eigen3
include_directories("usr/include/eigen3"
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)



# include myslam link_directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/myslam
  ${PROJECT_SOURCE_DIR}/myslam/include
  # Include directories for ament dependencies
  ${ament_INCLUDE_DIRS}
  )

set(PACKAGE_INCLUDE_DEPENDENCIES
  rclcpp
  rclpy
  std_msgs
  sensor_msgs

  glog
  OpenCV
  Eigen3
  Pangolin
  )

# adding myslam library
add_library(myslam_lib SHARED
  myslam/include/visual_odometry.h
  myslam/src/visual_odometry.cpp
  myslam/include/common_include.h
  myslam/include/config.h
  myslam/src/config.cpp
  myslam/include/dataset.h
  myslam/src/dataset.cpp
  myslam/include/frame.h
  myslam/src/frame.cpp
)

set_target_properties(myslam_lib PROPERTIES LINKER_LANGUAGE CXX)

ament_target_dependencies(myslam_lib
${PACKAGE_INCLUDE_DEPENDENCIES}

)


target_link_libraries(myslam_lib
  glog::glog  # glog link
)

add_executable(test_node
  src/test_node.cpp
)

ament_target_dependencies(test_node
 PUBLIC ${PACKAGE_INCLUDE_DEPENDENCIES}
)

target_link_libraries(test_node
 PUBLIC myslam_lib
)

set_target_properties(test_node PROPERTIES INSTALL_RPATH "\$ORIGIN")

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS test_node myslam_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)


ament_package()
