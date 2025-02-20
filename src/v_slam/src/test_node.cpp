
#include "rclcpp/rclcpp.hpp"
#include <iostream>

#include "visual_odometry.h"

int main() {
  std::string path = "path/to/config";
  auto node = std::make_shared<myslam::VO>(path);

  node->Init();
  node->Run();

  return 0;
}