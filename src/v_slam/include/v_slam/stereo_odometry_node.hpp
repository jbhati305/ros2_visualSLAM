#ifndef VISUAL_SLAM_STEREO_ODOMETRY_NODE_HPP
#define VISUAL_SLAM_STEREO_ODOMETRY_NODE_HPP

#include <memory>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sequence.h"
#include "sensor/camera.h"
#include "context.h"

class StereoOdometryNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    StereoOdometryNode(const std::string & node_name = "stereo_odometry_node");

    bool Init();
    void Run();

private:
    void loadGroundTruthPoses();
    void publishLeftImage(const sensor_msgs::msg::Image::SharedPtr & image);
    void publishRightImage(const sensor_msgs::msg::Image::SharedPtr & image);
    void publishPose(const Sophus::SE3d & pose);
    void visualizeContext(const Context & context);

    std::string path_root_;
    std::string calibration_file_;
    std::shared_ptr<Sequence> sequence_ = nullptr;
    std::vector<std::shared_ptr<Camera>> cameras_;

    std::shared_ptr<image_transport::ImageTransport> image_trans_;
    image_transport::CameraPublisher image_pub_00_;
    image_transport::CameraPublisher image_pub_01_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_truth_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_keypoints_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_features_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_optical_flow_;

    nav_msgs::msg::Path path_msg_;
    nav_msgs::msg::Path path_msg_truth_;

    bool initialized_ = false;
};

#endif // VISUAL_SLAM_STEREO_ODOMETRY_NODE_HPP
