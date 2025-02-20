/*
 * Description: Implements the Frame struct declared in frame.h. This file provides the 
 * functionality to create a new frame, assign it a unique id, set keyframe status, and 
 * initialize frame data such as stereo images, timestamps, and pose.
 */

#include "frame.h"

namespace myslam {

Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
    : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++;
    return new_frame;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}

} // namespace myslam