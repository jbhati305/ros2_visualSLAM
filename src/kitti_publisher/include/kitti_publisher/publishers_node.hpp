#ifndef PUBLISHERS_NODE_HPP_
#define PUBLISHERS_NODE_HPP_

#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <fstream>
#include <filesystem>
#include <vector>
#include <string>
#include <cstdlib>

// // TODO: Include the necessary header files
// #include "kitti_publisher/visibility.h"
// #include "kitti_publisher/WGS84toCartesian.hpp"

class KittiPublishersNode : public rclcpp::Node
{
    public:
        enum class PublisherType
        {
            POINT_CLOUD = 0,
            IMAGE_LEFT_GRAY = 1,
            IMAGE_RIGHT_GRAY = 2,
            IMAGE_LEFT_COLOR = 3,
            IMAGE_RIGHT_COLOR = 4,
            ODOMETRY = 5
        }

    KITTI_PUBLISHERS_NODE_PUBLIC KittiPublishersNode();

    std::string get_path(PublisherType publisher_type);
    std::vector<std::string> get_filenames(PublisherType publisher_type);
    void set_filenames(PublisherType publisher_type, std::vector<std::string> file_names);

    private:
        void on_timer_callback();

        void init_file_path();
        void create_publishers_data_file_names();
        std::vector<std::string> parse_file_data_into_string_array(std::string file_name, std::string delimiter);

        std::string mat_type2encoding(int mat_type);
        void convert_image_to_msg(sensor_msgs::msg::Image & msg, const std::string path);

        // Some necessary functions
        void prepare_navsatfix_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::NavSatFix &msg);
        void prepare_imu_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::Imu &msg);
        void prepare_marker_array_msg(std::vector<std::string> &oxts_tokenized_array, visualization_msgs::msg::MarkerArray &msg);
        void convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 & msg);

        size_t file_index_;

        rclcpp::TimerBase::SharedPtr timer_;

        // Create the publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_gray_left_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_gray_right_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_color_left_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_color_right_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_odometry_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_nav_sat_fix_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_array_;

        // Create the vectors to store the file names
        std::vector<std::string> file_names_point_cloud_;
        std::vector<std::string> file_names_image_gray_left_;
        std::vector<std::string> file_names_image_gray_right_;
        std::vector<std::string> file_names_image_color_left_;
        std::vector<std::string> file_names_image_color_right_;
        std::vector<std::string> file_names_oxts_;

        // Create the strings to store the file paths
        std::string path_point_cloud_;
        std::string path_image_gray_left_;
        std::string path_image_gray_right_;
        std::string path_image_color_left_;
        std::string path_image_color_right_;
        std::string path_oxts_;

};

#endif  // PUBLISHERS_NODE_HPP_