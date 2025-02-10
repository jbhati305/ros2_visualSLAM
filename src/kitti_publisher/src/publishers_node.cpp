#include <chrono>

#include "kitti_publisher/publishers_node.hpp"

using namespace cv;
using namespace std::chrono_literals;

KittiPublishersNode::KittiPublishersNode(): Node("publisher_node"), file_index_(0)
{
    publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud", 10);
    publisher_image_gray_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/left", 10);
    publisher_image_gray_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/gray/right", 10);
    publisher_image_color_left_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/left", 10);
    publisher_image_color_right_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image/color/right", 10);
    publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("kitti/imu", 10);
    publisher_nav_sat_fix_= this->create_publisher<sensor_msgs::msg::NavSatFix>("kitti/nav_sat_fix", 10);
    publisher_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("kitti/marker_array", 10);

    init_file_path();

    create_publishers_data_file_names();

    timer_ = create_wall_timer(100ms, std::bind(&KittiPublishersNode::on_timer_callback, this));
}

void KittiPublishersNode::init_file_path()
{
    path_point_cloud_ = "data/2011_09_26/2011_09_26_drive_0015_sync/velodyne_points/data/";
    path_image_gray_left_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_00/data/";
    path_image_gray_right_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_01/data/";
    path_image_color_left_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_02/data/";
    path_image_color_right_ = "data/2011_09_26/2011_09_26_drive_0015_sync/image_03/data/";
    path_oxts_ = "data/2011_09_26/2011_09_26_drive_0015_sync/oxts/data/";
}

void KittiPublishersNode::on_timer_callback()
{
    // 01- KITTI POINT CLOUDS2 MESSAGES START//
    sensor_msgs::msg::PointCloud2 point_cloud2_msg;
    convert_pcl_to_pointcloud2(point_cloud2_msg);
    // 01- KITTI POINT CLOUDS2 MESSAGES END//

    // 02- KITTI IMAGE MESSAGES START- gray_left(image_00), gray_right(image_01), color_left(image_02), color_right(image_03)//   
    auto image_message_gray_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_gray_left = path_image_gray_left_ + file_names_image_color_left_[file_index_];
    convert_image_to_msg(*image_message_gray_left, img_pat_gray_left);

    auto image_message_gray_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_gray_right = path_image_gray_right_ + file_names_image_color_right_[file_index_];
    convert_image_to_msg(*image_message_gray_right, img_pat_gray_right);

    auto image_message_color_left = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_left = path_image_color_right_ + file_names_image_color_left_[file_index_];
    convert_image_to_msg(*image_message_color_left, img_pat_color_left);

    auto image_message_color_right = std::make_unique<sensor_msgs::msg::Image>();
    std::string img_pat_color_right = path_image_color_right_ + file_names_image_color_right_[file_index_];
    convert_image_to_msg(*image_message_color_right, img_pat_color_right);
    // 02- KITTI IMAGE MESSAGES END // 

    // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE START//
    std::string oxts_file_name = path_oxts_ + file_names_oxts_[file_index_];
    const std::string delimiter = " ";
    std::vector<std::string> oxts_parsed_array = parse_file_data_into_string_array(oxts_file_name, delimiter);
    RCLCPP_INFO(this->get_logger(), "OxTs size: '%i'", oxts_parsed_array.size());

    auto nav_sat_fix_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
    prepare_navsatfix_msg(oxts_parsed_array , *nav_sat_fix_msg);

    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    prepare_imu_msg(oxts_parsed_array , *imu_msg);

    auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
    prepare_marker_array_msg(oxts_parsed_array , *marker_array_msg);
    // 03- KITTI OXTS to IMU, NAV & MARKERARRAY MESSAGE END//

    publisher_point_cloud_->publish(point_cloud2_msg);
    publisher_image_gray_left_->publish(std::move(image_message_gray_left));
    publisher_image_gray_right_->publish(std::move(image_message_gray_right));
    publisher_image_color_left_->publish(std::move(image_message_color_left));
    publisher_image_color_right_->publish(std::move(image_message_color_right));

    publisher_imu_->publish(std::move(imu_msg));
    publisher_nav_sat_fix_->publish(std::move(nav_sat_fix_msg));
    publisher_marker_array_->publish(std::move(marker_array_msg));

    file_index_++;
}

void KittiPublishersNode::convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg ){
    pcl::PointCloud<pcl::PointXYZI> cloud;

    std::string filePath = get_path(KittiPublishersNode::PublisherType::POINT_CLOUD) + file_names_point_cloud_[file_index_];
    std::fstream input(filePath, std::ios::in | std::ios::binary);
    if(!input.good()){
      RCLCPP_INFO(this->get_logger(), "Could not read Velodyne's point cloud. Check your file path!");
      exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    for (int i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud.push_back(point);
    }

    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
}