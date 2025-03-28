/*
 * Description: Implements the Config singleton class used for loading and
 * accessing configuration parameters from a file via OpenCV's FileStorage. This
 * file handles instantiating the Config object, loading a configuration file,
 *              error checking, and releasing the file resource upon
 * destruction.
 */

#include "config.h"

namespace myslam {

bool Config::SetParameterFile(const std::string &filename) {
  if (config_ == nullptr)
    config_ = std::shared_ptr<Config>(new Config);
  config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
  if (config_->file_.isOpened() == false) {
    LOG(ERROR) << "parameter file " << filename << " does not exist.";
    config_->file_.release();
    return false;
  }
  return true;
}

Config::~Config() {
  if (file_.isOpened())
    file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;
} // namespace myslam