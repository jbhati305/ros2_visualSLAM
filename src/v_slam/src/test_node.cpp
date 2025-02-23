#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "visual_odometry.h"



std::string getConfigFilePath(const std::string package_name, const std::string config_file) {
	std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
	return package_share_directory + "/config/" + config_file;
}



int main() {
	
	std::string path = getConfigFilePath("v_slam", "default.yaml");
	auto node = std::make_shared<myslam::VO>(path);

	node->Init();
	node->Run();

	return 0;
}