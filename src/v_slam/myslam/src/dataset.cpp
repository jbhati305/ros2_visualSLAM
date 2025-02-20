#include "dataset.h"

#include <boost/format.hpp> // for formatting strings
#include <fstream>
#include <opencv2/opencv.hpp>


#include "frame.h"

using namespace std;

namespace myslam {

    // Constructor: Stores the provided dataset path for future file I/O operations.
    Dataset::Dataset(const std::string &dataset_path)
        : dataset_path_(dataset_path) {}

    // Init(): Reads calibration parameters and initializes the camera 
    // configuration for the SLAM system.
    /*
       Math & Functionality:
       ----------------------
       1. Open the calibration file "calib.txt" which should contain the 
          camera names and projection matrices.
       2. For each camera:
           a. Reads a 3x4 projection matrix (12 values).
           b. Constructs an intrinsic matrix K by extracting the focal lengths 
              and principal point values from the projection matrix.
           c. Extracts a translation vector t from the 4th column.
           d. Computes the transformed translation t by calculating K.inverse() * t.
           e. Scales the intrinsic matrix K by a factor (in this case 0.5) to account 
              for image resizing.
           f. Creates a new Camera object with these parameters.
    */
    bool Dataset::Init() {
        // load camera intrinsics
        ifstream fin(dataset_path_ + "/calib.txt");
        if (!fin) {
            LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
            return false;
        }
        for (int i = 0; i < 4; ++i) {
            // Read a 3-character camera name (unused here)
            char camera_name[3];
            for (int k = 0; k < 3; ++k) {
                fin >> camera_name[k];
            }
            // Read the 3x4 projection matrix (12 values)
            double projection_data[12];
            for (int k = 0; k < 12; ++k) {
                fin >> projection_data[k];
            }
            // Construct the intrinsic matrix K from the projection matrix.
            // K = [fx,  0, cx;
            //       0, fy, cy;
            //       0,  0,  1]
            Mat33 K;
            K << projection_data[0], projection_data[1], projection_data[2],
                projection_data[4], projection_data[5], projection_data[6],
                projection_data[8], projection_data[9], projection_data[10];
            // Extract the translation vector t (camera extrinsics)
            Vec3 t;
            t << projection_data[3], projection_data[7], projection_data[11];
            // Transform t using the inverse of K (solving for translation in normalized coordinates)
            t = K.inverse() * t;
            // Scale the intrinsic matrix, corresponding to a 50% image resize later on.
            K = K * 0.5;
            
            // Create a new Camera object using the intrinsic parameters.
            // The parameters include focal lengths, principal points, baseline (using norm of t), and pose.
            // TODO CAMERA PTR
            // Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
            //                                   t.norm(), SE3(SO3(), t)));
            // cameras_.push_back(new_camera);
            LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
        }
        fin.close();
        current_image_index_ = 0;
        return true;
    }

    // NextFrame(): Loads the next stereo frame from the dataset.
    /*
       Functionality:
       --------------
       1. Construct file paths for left and right images using a formatted string.
       2. Read images in grayscale mode.
       3. Verify if images are loaded; log a warning if missing.
       4. Resize images by scaling them down (factor 0.5) to match calibration scaling.
       5. Create a new Frame, assign the images, and increment the image index.
    */

  
    Frame::Ptr Dataset::NextFrame() {
        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat image_left, image_right;
        // read images
        image_left =
            cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
        image_right =
            cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
    
        if (image_left.data == nullptr || image_right.data == nullptr) {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }
    
        cv::Mat image_left_resized, image_right_resized;
        cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    
        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = image_left_resized;
        new_frame->right_img_ = image_right_resized;
        current_image_index_++;
        return new_frame;
    }
}