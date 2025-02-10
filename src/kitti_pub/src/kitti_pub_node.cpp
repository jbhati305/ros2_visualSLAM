#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>

namespace fs = std::filesystem;

// ...existing code or includes if any...

class KittiPubNode : public rclcpp::Node {
public:
    KittiPubNode() : Node("kitti_pub_node"), current_file_index_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti_lidar", 10);
        
        // Load binary file paths from the KITTI data directory
        std::string data_dir = "/home/jitesh/ros2_visualSLAM/data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data";
        for (const auto &entry : fs::directory_iterator(data_dir))
        {
            if (entry.path().extension() == ".bin")
                file_list_.push_back(entry.path().string());
        }
        std::sort(file_list_.begin(), file_list_.end());
        

        // Open the file containing timestamps.
        std::ifstream infile("/home/jitesh/ros2_visualSLAM/data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/timestamps.txt");
        if (!infile.is_open()) {
            std::cerr << "Error opening file." << std::endl;
            // return 1;
        }
        std::string line;
        while (std::getline(infile, line)) {
            // Make sure the line is long enough to contain the expected format (at least up to seconds and fraction)
            if (line.size() < 19) {continue;}
            // Extract the minute component from positions 14 and 15: "HH:MM:SS.XXXX"
            std::string minuteStr = line.substr(14, 2); // e.g., "02"
            int minute = std::stoi(minuteStr);
            // Extract the seconds component with all fractional digits from index 17 onward.
            // For example, for "25.951199337", this extracts the full "25.951199337" string.
            std::string secondsStr = line.substr(17);
            double seconds = std::stod(secondsStr);
            // If desired, compute the total seconds relative to the beginning of the minute.
            // This adds the whole minute part (minute * 60) to the floating-point seconds value.
            double totalSeconds = minute * 60 + seconds;
            timestamps_.push_back(totalSeconds);
        }
        infile.close();

        // Start with initial timer (delay 0)
        schedule_timer(0.0);
    }
private:
    void schedule_timer(double delay_sec) {
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(delay_sec),
            std::bind(&KittiPubNode::timer_callback, this)
        );
    }
    
    void timer_callback() {
        // Check if all files published
        if(current_file_index_ >= file_list_.size()){
            RCLCPP_INFO(this->get_logger(), "All files published");
            return;
        }

        // Open the current binary file
        std::string file_path = file_list_[current_file_index_];
        std::ifstream infile(file_path, std::ios::binary | std::ios::ate);
        if(!infile.is_open()){
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            current_file_index_++;
            schedule_timer(1.0);
            return;
        }
        auto file_size = infile.tellg();
        infile.seekg(0, std::ios::beg);
        std::vector<char> buffer(file_size);
        infile.read(buffer.data(), file_size);
        infile.close();
        
        // Each point is 4 floats (x,y,z,intensity)
        size_t num_points = file_size / (4 * sizeof(float));

        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.height = 1;
        msg.width = num_points;
        msg.is_bigendian = false;
        msg.point_step = 4 * sizeof(float);
        msg.row_step = msg.point_step * num_points;
        msg.is_dense = true;
        msg.data = std::vector<unsigned char>(buffer.begin(), buffer.end());

        // Define point fields (x, y, z, intensity)
        msg.fields.resize(4);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[0].count = 1;

        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[1].count = 1;

        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[2].count = 1;

        msg.fields[3].name = "intensity";
        msg.fields[3].offset = 12;
        msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[3].count = 1;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published pointcloud2 (%zu points) from file: %s", num_points, file_path.c_str());

        // Compute delay based on the difference between start times of the next and current frame
        double delay = 1.0;
        if((current_file_index_ + 1) < timestamps_.size()){
            delay = timestamps_[current_file_index_ + 1] - timestamps_[current_file_index_];
            
        }
        current_file_index_++;
        schedule_timer(delay);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> file_list_;
    std::vector<double> timestamps_;           // kept for reference if needed
    
    size_t current_file_index_;
    
// ...existing code...
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KittiPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
