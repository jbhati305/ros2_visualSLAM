#include "kitti_pub/kitti_pub_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cstdlib>

namespace fs = std::filesystem;

// Helper: Loads file paths from a directory into the vector associated with the publisher type.
void KittiPubNode::load_files(PublisherType publisher, const std::string &dir)
{
    std::vector<std::string>* target = nullptr;
    switch (publisher) {
        case PublisherType::POINT_CLOUD:   target = &file_list_; break;
        case PublisherType::IMAGE_LEFT_GRAY: target = &file_names_image_gray_left_; break;
        case PublisherType::IMAGE_RIGHT_GRAY:target = &file_names_image_gray_right_; break;
        case PublisherType::IMAGE_LEFT_COLOR:target = &file_names_image_color_left_; break;
        case PublisherType::IMAGE_RIGHT_COLOR:target = &file_names_image_color_right_; break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unsupported publisher type");
            return;
    }
    for (const auto &entry : fs::directory_iterator(dir)) {
        if (publisher == PublisherType::POINT_CLOUD) {
            if (entry.path().extension() == ".bin")
                target->push_back(entry.path().string());
        } else {  // For image files.
            if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg")
                target->push_back(entry.path().string());
        }
    }
    std::sort(target->begin(), target->end());
}

KittiPubNode::KittiPubNode() : Node("kitti_pub_node"), current_file_index_(0)
{
    // Create publishers
    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/point_cloud2", 10);
    image_gray_left_publisher_  = this->create_publisher<sensor_msgs::msg::Image>("kitti/image_gray/left", 10);
    image_gray_right_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image_gray/right", 10);
    image_color_left_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("kitti/image_color/left", 10);
    image_color_right_publisher_= this->create_publisher<sensor_msgs::msg::Image>("kitti/image_color/right", 10);
    
    init_file_path();
    // Load files using the helper for each type
    load_files(PublisherType::POINT_CLOUD,      path_point_cloud_);
    load_files(PublisherType::IMAGE_LEFT_GRAY,    path_image_gray_left_);
    load_files(PublisherType::IMAGE_RIGHT_GRAY,   path_image_gray_right_);
    load_files(PublisherType::IMAGE_LEFT_COLOR,   path_image_color_left_);
    load_files(PublisherType::IMAGE_RIGHT_COLOR,  path_image_color_right_);
    
    read_timestamps();
    schedule_timer(0.0);
}

void KittiPubNode::read_timestamps()
{
    std::ifstream infile(timestamps_file_path_);
    if (!infile.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open timestamps file: %s", timestamps_file_path_.c_str());
        std::exit(EXIT_FAILURE);
    }
    std::string line;
    while (std::getline(infile, line)) {
        if (line.size() < 19) continue;  // Skip short lines
        int minute = std::stoi(line.substr(14, 2));
        double seconds = std::stod(line.substr(17));
        timestamps_.push_back(minute * 60 + seconds);
    }
}

void KittiPubNode::init_file_path()
{
    // Set directory paths (update if needed)
    path_point_cloud_      = "data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data";
    path_image_gray_left_  = "data/2011_09_26/2011_09_26_drive_0001_sync/image_00/data";
    path_image_gray_right_ = "data/2011_09_26/2011_09_26_drive_0001_sync/image_01/data";
    path_image_color_left_ = "data/2011_09_26/2011_09_26_drive_0001_sync/image_02/data";
    path_image_color_right_= "data/2011_09_26/2011_09_26_drive_0001_sync/image_03/data";
    timestamps_file_path_  = "data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/timestamps.txt";
}

void KittiPubNode::schedule_timer(double delay_sec)
{
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(delay_sec),
        std::bind(&KittiPubNode::timer_callback, this)
    );
}

void KittiPubNode::publish_point_cloud()
{
    std::string file_path = file_list_[current_file_index_];
    std::ifstream infile(file_path, std::ios::binary | std::ios::ate);
    if (!infile.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", file_path.c_str());
        return;
    }
    auto file_size = infile.tellg();
    infile.seekg(0, std::ios::beg);
    std::vector<char> buffer(file_size);
    infile.read(buffer.data(), file_size);
    
    size_t num_points = file_size / (4 * sizeof(float));
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.height = 1;
    msg.width = num_points;
    msg.point_step = 4 * sizeof(float);
    msg.row_step = msg.point_step * num_points;
    msg.data.assign(buffer.begin(), buffer.end());
    
    // Replace brace initializer with push_back for each field.
    msg.fields.clear();
    
    sensor_msgs::msg::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    msg.fields.push_back(field);
    
    field.name = "y";
    field.offset = 4;
    msg.fields.push_back(field);
    
    field.name = "z";
    field.offset = 8;
    msg.fields.push_back(field);
    
    field.name = "intensity";
    field.offset = 12;
    msg.fields.push_back(field);

    point_cloud_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published point cloud (%zu points) from: %s", num_points, file_path.c_str());
}

std::shared_ptr<sensor_msgs::msg::Image> KittiPubNode::convert_image_to_msg(const std::string &image_path, bool is_color)
{
    int flag = is_color ? cv::IMREAD_COLOR : cv::IMREAD_GRAYSCALE;
    cv::Mat image = cv::imread(image_path, flag);
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path.c_str());
        return nullptr;
    }
    std::string encoding = is_color ? "bgr8" : "mono8";
    return cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
}

void KittiPubNode::publish_images()
{
    if (file_names_image_gray_left_.size() > current_file_index_) {
        if (auto msg = convert_image_to_msg(file_names_image_gray_left_[current_file_index_], false))
            image_gray_left_publisher_->publish(*msg);
    }
    if (file_names_image_gray_right_.size() > current_file_index_) {
        if (auto msg = convert_image_to_msg(file_names_image_gray_right_[current_file_index_], false))
            image_gray_right_publisher_->publish(*msg);
    }
    if (file_names_image_color_left_.size() > current_file_index_) {
        if (auto msg = convert_image_to_msg(file_names_image_color_left_[current_file_index_], true))
            image_color_left_publisher_->publish(*msg);
    }
    if (file_names_image_color_right_.size() > current_file_index_) {
        if (auto msg = convert_image_to_msg(file_names_image_color_right_[current_file_index_], true))
            image_color_right_publisher_->publish(*msg);
    }
}

void KittiPubNode::timer_callback()
{
    if (current_file_index_ >= file_list_.size()) {
        RCLCPP_INFO(this->get_logger(), "All files published");
        return;
    }
    publish_point_cloud();
    publish_images();
    double delay = 1.0;
    if ((current_file_index_ + 1) < timestamps_.size())
        delay = timestamps_[current_file_index_ + 1] - timestamps_[current_file_index_];
    current_file_index_++;
    schedule_timer(delay);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KittiPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
