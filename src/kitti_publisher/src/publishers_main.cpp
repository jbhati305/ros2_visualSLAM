#include <memory>
#include "kitti_publisher/publishers_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<KittiPublishersNode>());

  rclcpp::shutdown();

  return 0;
}