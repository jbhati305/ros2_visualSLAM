/*
Description: Declares the Dataset class.
            It's a class that will be used to load and access the data
            needed for the SLAM system.
*/

#pragma once
#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

// include some headers
#include "common_include.h"
#include "frame.h"

#include "camera.h"

namespace myslam {
class Dataset {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Dataset> Ptr;
  Dataset(const std::string &dataset_path);
  bool Init();

  Frame::Ptr NextFrame();

  Camera::Ptr GetCamera(int camera_id) const { return cameras_.at(camera_id); }

private:
  std::string dataset_path_;
  int current_image_index_ = 0;

  std::vector<Camera::Ptr> cameras_;
};

} // namespace myslam

#endif // MYSLAM_DATASET_H