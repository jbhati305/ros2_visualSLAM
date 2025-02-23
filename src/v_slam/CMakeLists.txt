cmake_minimum_required(VERSION 3.8)
project(v_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Enforce use of C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")


list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/myslam/cmake_modules)

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

find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
find_package(GFlags REQUIRED)
find_package(GTest REQUIRED)


# include eigen3
include_directories("usr/include/eigen3"
  "usr/include/opencv4"
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${GFLAGS_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
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
  myslam/include/camera.h
  myslam/src/camera.cpp
  myslam/include/frontend.h
  myslam/src/frontend.cpp
  myslam/include/feature.h
  myslam/src/feature.cpp
  myslam/include/g2o.h
  myslam/include/mappoint.h 
  myslam/src/mappoint.cpp
  myslam/include/map.h
  myslam/src/map.cpp
  myslam/include/backend.h
  myslam/src/backend.cpp
  myslam/include/viewer.h
  myslam/src/viewer.cpp
  myslam/include/algorithm.h
  
)

set_target_properties(myslam_lib PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(myslam_lib
  ${THIRD_PARTY_LIBS}
)

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
