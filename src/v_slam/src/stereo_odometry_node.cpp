#include "v_slam/stereo_odometry_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>

// Constructor: Initialize publishers and set up file paths
StereoOdometryNode::StereoOdometryNode(const std::string & node_name) : Node(node_name),
    path_root_("/data/kitti/odometry/dataset/sequences/00/"),
    calibration_file_(path_root_ + "/calib.txt")
{
    // Create publishers using Node::create_publisher and image_transport from ROS2
    image_trans_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    image_pub_00_ = image_trans_->advertiseCamera("grayscale/left/image", 1);
    image_pub_01_ = image_trans_->advertiseCamera("grayscale/right/image", 1);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);
    path_publisher_truth_ = this->create_publisher<nav_msgs::msg::Path>("path_truth", 1);

    image_pub_keypoints_ = this->create_publisher<sensor_msgs::msg::Image>("keypoints", 1);
    image_pub_features_ = this->create_publisher<sensor_msgs::msg::Image>("features", 1);
    image_pub_optical_flow_ = this->create_publisher<sensor_msgs::msg::Image>("optical_flow", 1);
}

// Init: Load calibration data and perform necessary initialization.
bool StereoOdometryNode::Init() {
    sequence_ = std::make_shared<Sequence>(path_root_);
    sequence_->Init();
    std::ifstream fin(calibration_file_);
    if (!fin) {return false;}
    for (int i = 0; i < 4; ++i) {
        char camera_name[3];
        // ...existing code to read camera name...
        for (int k = 0; k < 3; ++k) { fin >> camera_name[k]; }
        double P[12];
        for (int k = 0; k < 12; ++k) { fin >> P[k]; }
        std::shared_ptr<Camera> camera = std::make_shared<Camera>(P[0], P[5], P[2], P[6], P[3]);
        cameras_.push_back(camera);
    }
    fin.close();
    initialized_ = true;
    return true;
}

// Run: Main processing loop. (Simulated for demonstration purposes)
void StereoOdometryNode::Run() {
    auto matcher = std::make_shared<Matcher>();
    auto tracker = std::make_shared<Tracker>();
    std::shared_ptr<Estimation> estimation = std::make_shared<Estimation>();
    auto map = std::make_shared<Map>();

    auto triangulator = std::make_shared<Triangulator>(cameras_[0], cameras_[1]);
    triangulator->setMap(map);

    auto backend = std::make_shared<Backend>();
    backend->setCameras(cameras_[0], cameras_[1]);
    backend->setMap(map);

    auto frontend = std::make_shared<Frontend>();
    frontend->setCameras(cameras_[0], cameras_[1]);
    frontend->setMap(map);
    frontend->setMatcher(matcher);
    frontend->setTracker(tracker);
    frontend->setTriangulator(triangulator);
    frontend->setEstimation(estimation);
    frontend->setBackend(backend);

    loadGroundTruthPoses();

    // Main loop: Read stereo pairs from the sequence and publish messages.
    std::shared_ptr<Sequence::StereoPair> prev_element = nullptr;
    for (auto element : sequence_->elements_) {
        if (prev_element) {
            double delay_seconds = element->timestamp_ - prev_element->timestamp_;
            if (delay_seconds > 0.0) {
                auto microseconds = static_cast<unsigned int>(delay_seconds * 1e6);
                std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
            }
        }
        cv::Mat image_00 = cv::imread(element->image_00_, cv::IMREAD_GRAYSCALE);
        auto image_msg_00 = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_00).toImageMsg();
        image_msg_00->header.frame_id = "base_link";

        cv::Mat image_01 = cv::imread(element->image_01_, cv::IMREAD_GRAYSCALE);
        auto image_msg_01 = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_01).toImageMsg();
        image_msg_01->header.frame_id = "base_link";

        frontend->pushback(image_00, image_01);
        Sophus::SE3d pose = frontend->getPose();

        publishLeftImage(image_msg_00);
        publishRightImage(image_msg_01);
        publishPose(pose);

        visualizeContext(frontend->getContext());
        rclcpp::spin_some(shared_from_this());
        prev_element = element;
    }
    backend->terminate();
}

// Publish left image message
void StereoOdometryNode::publishLeftImage(const sensor_msgs::msg::Image::SharedPtr & image) {
      // Create a dummy CameraInfo message
      auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
      camera_info_msg->header.frame_id = image->header.frame_id;
      image_pub_00_.publish(*image, *camera_info_msg);
}

// Publish right image message
void StereoOdometryNode::publishRightImage(const sensor_msgs::msg::Image::SharedPtr & image) {
    auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    camera_info_msg->header.frame_id = image->header.frame_id;
    image_pub_01_.publish(*image, *camera_info_msg);
}

// Convert a Sophus::SE3d pose to a PoseStamped message and publish it.
void StereoOdometryNode::publishPose(const Sophus::SE3d & pose) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose.position.x = pose.translation().x();
    pose_msg.pose.position.y = pose.translation().z();
    pose_msg.pose.position.z = pose.translation().y();
    // For now orientations are zero; update this conversion as needed.
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
    pose_msg.pose.orientation.w = 0.0;

    pose_pub_->publish(pose_msg);
    pose_msg.header.stamp = this->now();
    path_msg_.header.frame_id = pose_msg.header.frame_id;
    path_msg_.poses.push_back(pose_msg);
    path_publisher_->publish(path_msg_);
    path_publisher_truth_->publish(path_msg_truth_);
}

// Visualize context (e.g., features, keypoints). Currently logs visualization.
void StereoOdometryNode::visualizeContext(const Context & context) {
    cv::Mat kpts_img;
    Vizualization::prepareKeypointsVisual(context, kpts_img);
    auto cv_ptr_kpts = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", kpts_img).toImageMsg();
    cv_ptr_kpts->header.frame_id = "base_link";
    image_pub_keypoints_->publish(*cv_ptr_kpts);

    cv::Mat feat_img;
    Vizualization::prepareFeaturesVisual(context, feat_img);
    auto cv_ptr_feats = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", feat_img).toImageMsg();
    cv_ptr_feats->header.frame_id = "base_link";
    image_pub_features_->publish(*cv_ptr_feats);

    cv::Mat flow_img;
    Vizualization::prepareOpticalFlowVizual(context, flow_img);
    auto cv_ptr_flow = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", flow_img).toImageMsg();
    cv_ptr_flow->header.frame_id = "base_link";
    image_pub_optical_flow_->publish(*cv_ptr_flow);
}

// Load ground truth poses from a file (if applicable)
void StereoOdometryNode::loadGroundTruthPoses() {
    std::string filepath = "/data/kitti/odometry/dataset/poses/";
    const std::string filename = filepath + "/00.txt";
    std::ifstream file(filename);
    if(!file){
        throw std::runtime_error("Could not load timestamps file: " + filepath);
    }
    path_msg_truth_.poses.clear();
    std::string line;
    while(std::getline(file, line)){
        std::istringstream input(line);
        cv::Mat pose = cv::Mat_<double>(3,4);
        // ...existing code to read pose...
        for (int i = 0; i < 12; ++i) {
            input >> pose.at<double>(i/4, i % 4);
        }
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = pose.at<double>(0,3);
        pose_msg.pose.position.y = pose.at<double>(2,3);
        pose_msg.pose.position.z = pose.at<double>(1,3);
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 0.0;
        path_msg_truth_.poses.push_back(pose_msg);
    }
    file.close();
}
