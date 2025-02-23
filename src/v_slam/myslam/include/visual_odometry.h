#pragma once

#include <iostream>
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "common_include.h"
#include "dataset.h"
#include "frontend.h"
#include "backend.h"
#include "viewer.h"

namespace myslam {
class VO {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef std::shared_ptr<VO> Ptr;

 
	VO(std::string &config_path);

	// Initialization
	bool Init();

	// start VO in the dataset
	void Run();

	// Make a step forward in dataset
	bool Step();

	// frontend status

	FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
	bool inited_ = false;
	std::string config_file_path_;
	Map::Ptr map_ = nullptr;


	Frontend::Ptr frontend_ = nullptr;
	Backend::Ptr backend_ = nullptr;
	Viewer::Ptr viewer_ = nullptr;

	Dataset::Ptr dataset_ = nullptr;

}; // class VO

} // namespace myslam

#endif // MYSLAM_VISUAL_ODOMETRY_H