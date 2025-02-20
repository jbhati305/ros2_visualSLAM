/*
Description:    Declares the Frame struct which represents a single frame in the SLAM system.
                It contains stereo images, pose information, timestamps, and associated features.
                The file also declares thread-safe functions to access and update the frame pose.
 */

#pragma once

#include <memory>
#include <mutex>
#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "common_include.h"
// TODO CAMERA.H
// #include "camera.h"

namespace myslam {

    // forward declare
    struct MapPoint;
    struct Feature;

    struct Frame{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;         // id of this frame
        unsigned long keyframe_id_ = 0;  // id of the keyframe
        bool is_keyframe_ = false;       // whether a keyframe
        double time_stamp_;              // when it is recorded
        SE3 pose_;
        cv::Mat left_img_, right_img_;   // stereo images

        // extracted features in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // corresponding features in right image, set to nullptr if no corresponding
        std::vector<std::shared_ptr<Feature>> features_right_;

    public:
        Frame(){}

        Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

        SE3 Pose() const { 
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;   
        }

        void SetPose(const SE3 &pose) {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        void SetKeyFrame();

        static std::shared_ptr<Frame> CreateFrame();
    private:
        mutable std::mutex pose_mutex_;
    };

}

#endif // MYSLAM_FRAME_H