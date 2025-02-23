/*

	* Description: Declares the Frontend class responsible for the visual odometry component

	* in the SLAM system. The Frontend processes incoming frames (from stereo cameras),
	tracks
 * features, estimates the current frame's pose,
	and determines whether a frame should be marked
 * as a keyframe. It interacts with the Map, Backend,
	and Viewer components to build and refine
 * the overall SLAM solution.
 *
 * Key Functions:
 *  - AddFrame(Frame::Ptr frame):

	*      Processes a new frame by tracking features and estimating the camera motion.
 *
 *  - Track():
 *      Performs feature tracking on the incoming frame.
 *
 *  - TrackLastFrame():

	*      Tracks features from the last frame; returns the number of tracked points.
 *
 *  - EstimateCurrentPose():

	*      Estimates the current frame’s pose using optimization; returns the number of inliers.
 *
 *  - InsertKeyframe():

	*      Determines if the current frame should be a keyframe and inserts it into the backend.
 *
 *  - StereoInit():
 *      Initializes the SLAM system using stereo images in the first frame.
 *
 *  - DetectFeatures():
 *      Detects features (keypoints) in the left image of the current frame.
 *
 *  - FindFeaturesInRight():

	*      Finds corresponding features in the right image based on detected keypoints.
 *
 *  - BuildInitMap():
 *      Constructs an initial map from the first frame.
 *
 *  - TriangulateNewPoints():
 *      Triangulates 2D points from stereo correspondences to get 3D positions.
 *
 *  - SetObservationsForKeyFrame():
 *      Updates map observations with the new keyframe data.
 */

#pragma once
#include <memory>
#ifndef MYSLAM_FRONTEND_H
# define MYSLAM_FRONTEND_H

#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/core/core.hpp>
# include "common_include.h"
# include "frame.h"
# include "map.h"
#include "backend.h"
#include "viewer.h"

namespace myslam
{
class	Backend;
class	Viewer;

enum class FrontendStatus
{
	INITING,
	TRACKING_GOOD,
	TRACKING_BAD,
	LOST
};

class Frontend
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef std::shared_ptr<Frontend> Ptr;

	Frontend();

	// Processes a new frame and returns true on success.
	bool AddFrame(Frame::Ptr frame);

	void SetMap(Map::Ptr map)
	{
		map_ = map;
	}

	void SetBackend(Backend::Ptr backend)
	{
		backend_ = backend;
	}

	void SetViewer(Viewer::Ptr viewer)
	{
		viewer_ = viewer;
	}

	FrontendStatus GetStatus() const
	{
		return (status_);
	}

	void SetCamera(Camera::Ptr left, Camera::Ptr right)
	{
		camera_left_ = left;
		camera_right_ = right;
	}

  private:
	// return true if success
	bool Track(); // Feature tracking for the current frame.
	bool Reset(); // Resets the frontend if tracking fails.

	// track with last frame
	// return num of tracked points
	int TrackLastFrame(); // Tracks features from the last frame; returns count.

	// Estimate current frame's pose
	// return num of inliers
	int EstimateCurrentPose();
		// Estimates current pose; returns number of inliers.

	// Set current frame as a keyframe and insert it into backend
	// return true if keyframe is inserted
	bool InsertKeyframe(); // Inserts frame as keyframe if necessary.

	// Try init the frontend with stereo images of current frame
	// return true if success
	bool StereoInit(); // Initializes the system from the first stereo pair.

	// Detect features in the left image of current frame
	// return num of features detected( keypoints detected)
	int DetectFeatures(); // Detects keypoints in the left image; returns count.

	// Find the corresponding features in the right image of current frame
	// return num of features found
	int FindFeaturesInRight();
		// Finds corresponding features in the right image; returns count.

	// Build initial map with the single image
	// return true if success
	bool BuildInitMap(); // Builds the initial map; returns success.

	// Triangulate the 2D points in the left and right image
	// return num of triangulated points
	int TriangulateNewPoints();
		// Triangulates 3D points from stereo images; returns count.

	// Set the feature in keyframe as new observation in the map
	void SetObservationsForKeyFrame();
		// Updates the map observations with new keyframe data.

	// Data
	FrontendStatus status_ = FrontendStatus::INITING;

	Frame::Ptr current_frame_ = nullptr;
	Frame::Ptr last_frame_ = nullptr;
	Camera::Ptr camera_left_ = nullptr;
	Camera::Ptr camera_right_ = nullptr;

	Map::Ptr map_ = nullptr;
	std::shared_ptr<Backend> backend_ = nullptr;
	std::shared_ptr<Viewer> viewer_ = nullptr;

	SE3 relative_motion_;

	// inlier threshold, for checking new points
	int tracking_inliers_ = 0;

	// Parameters
	int num_features_ = 200;
	int num_features_init_ = 100;
	int num_features_tracking_ = 50;
	int num_features_tracking_bad_ = 20;
	int num_features_needed_for_keyframe_ = 80;

	// utilities
	// feature detector and descriptor in opencv
	// cv::Ptr<cv::GFTTDetector> gftt_;
	cv::Ptr<cv::ORB> orb_;
	
	// gftt_ = cv::GFTTDetector::create();

};
} // namespace myslam

#endif // MYSLAM_FRONTEND_H