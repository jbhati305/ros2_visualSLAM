#ifndef KITTI_PUB_KITTI_PUB_NODE_HPP_
#define KITTI_PUB_KITTI_PUB_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vector>
#include <string>

class KittiPubNode : public rclcpp::Node {
public:
    enum class PublisherType 
  { 
      POINT_CLOUD = 0,
      IMAGE_LEFT_GRAY = 1, 
      IMAGE_RIGHT_GRAY = 2,  
      IMAGE_LEFT_COLOR = 3, 
      IMAGE_RIGHT_COLOR = 4, 
      ODOMETRY = 5,
      TIMESTAMPS = 6   // New type for timestamps file
  };

    KittiPubNode();

    std::string get_path(PublisherType publisher_type);
    std::vector<std::string> get_filenames(PublisherType publisher_type);
    void set_filenames(PublisherType publisher_type, std::vector<std::string> file_names);

    // New function declarations:
    std::shared_ptr<sensor_msgs::msg::Image> convert_image_to_msg(const std::string &image_path, bool is_color);
    void publish_images();
    void load_files(PublisherType publisher, const std::string &dir);  // New function declaration

private:
    void timer_callback();
    void schedule_timer(double delay_sec);
    void init_file_path();
    void publish_point_cloud();  // New function for publishing point cloud
    void read_timestamps();  // New function declaration

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_gray_left_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_gray_right_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_color_left_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_color_right_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> file_list_;
    std::vector<double> timestamps_;
    size_t current_file_index_;

    std::vector<std::string> file_names_point_cloud_;
    std::vector<std::string> file_names_image_gray_left_;
    std::vector<std::string> file_names_image_gray_right_;
    std::vector<std::string> file_names_image_color_left_;
    std::vector<std::string> file_names_image_color_right_;
    std::vector<std::string> file_names_oxts_;

    std::string path_point_cloud_;
    std::string path_image_gray_left_;
    std::string path_image_gray_right_;
    std::string path_image_color_left_;
    std::string path_image_color_right_;
    std::string path_oxts_;
    std::string timestamps_file_path_;  // New member for timestamps file path
};

#endif  // KITTI_PUB_KITTI_PUB_NODE_HPP_

