#include "config.h"
#include "frontend.h"
#include <opencv2/opencv.hpp>

#include "feature.h"
#include "g2o.h"

#include "algorithm.h"
#include "backend.h"
#include "viewer.h"
#include "map.h"

namespace myslam
{
Frontend::Frontend()
{
	gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 10);
	num_features_init_ = Config::Get<int>("num_features_init");
	num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame)
{
	current_frame_ = frame;
	switch (status_)
	{
	case FrontendStatus::INITING:
		StereoInit();
		break ;
	case FrontendStatus::TRACKING_GOOD:
	case FrontendStatus::TRACKING_BAD:
		Track();
		break ;
	case FrontendStatus::LOST:
		Reset();
		break ;
	}
	last_frame_ = current_frame_;
	return (true);
}

bool Frontend::Track()
{
	int	num_track_last;

	if (last_frame_)
	{
		current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
	}
	num_track_last = TrackLastFrame();
	tracking_inliers_ = EstimateCurrentPose();
	if (tracking_inliers_ > num_features_tracking_)
	{
		// tracking good
		status_ = FrontendStatus::TRACKING_GOOD;
	}
	else if (tracking_inliers_ > num_features_tracking_bad_)
	{
		// tracking bad
		status_ = FrontendStatus::TRACKING_BAD;
	}
	else
	{
		// lost
		status_ = FrontendStatus::LOST;
	}
	InsertKeyframe();
	relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
	if (viewer_)
		viewer_->AddCurrentFrame(current_frame_);
	return (true);
}

bool Frontend::InsertKeyframe()
{
	if (tracking_inliers_ >= num_features_needed_for_keyframe_)
	{
		// still have enough features, don't insert keyframe
		return (false);
	}
	// current frame is a new keyframe
	current_frame_->SetKeyFrame();
	map_->InsertKeyFrame(current_frame_);
	LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe " << current_frame_->keyframe_id_;
	SetObservationsForKeyFrame();
	DetectFeatures(); // detect new features
	// track in right image
	FindFeaturesInRight();
	// triangulate map points
	TriangulateNewPoints();
	// update backend because we have a new keyframe
	backend_->UpdateMap();
	if (viewer_)
		viewer_->UpdateMap();
	return (true);
}

void Frontend::SetObservationsForKeyFrame()
{
	std::shared_ptr<MapPoint> mp = nullptr;

	for (auto &feat : current_frame_->features_left_)
	{
		mp = feat->map_point_.lock();
		if (mp)
			mp->AddObservation(feat);
	}
}

int Frontend::TriangulateNewPoints()
{
	SE3		current_pose_Twc;
	int		cnt_triangulated_pts;
	Vec3	pworld;
	std::shared_ptr<MapPoint> new_map_point = nullptr;

	// get the poses of the two cameras
	std::vector<SE3> poses{camera_left_->Pose(), camera_right_->Pose()};


	current_pose_Twc = current_frame_->Pose().inverse();
	cnt_triangulated_pts = 0;

	for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
	{
		// this if is checking if the point is not triangulated and if the right feature is not null
		if (current_frame_->features_left_[i]->map_point_.expired()
			&& current_frame_->features_right_[i] != nullptr)
		{
			// vector for the 2D points in the left and right image of the current frame
			std::vector<Vec3> points{camera_left_->pixel2camera(Vec2(current_frame_->features_left_[i]->position_.pt.x,
																	 current_frame_->features_left_[i]->position_.pt.y)),
									camera_right_->pixel2camera(Vec2(current_frame_->features_right_[i]->position_.pt.x,
																	current_frame_->features_right_[i]->position_.pt.y))};
			// vector for the 3D point in the world frame
			pworld = Vec3::Zero();

			// Checking if the triangulation is successful and z coodinate is positive
			if (triangulation(poses, points, pworld) && pworld[2] > 0)
			{
				new_map_point = MapPoint::CreateNewMappoint();
				pworld = current_pose_Twc * pworld;
				new_map_point->SetPos(pworld);
				new_map_point->AddObservation(current_frame_->features_left_[i]);
				new_map_point->AddObservation(current_frame_->features_right_[i]);
				current_frame_->features_left_[i]->map_point_ = new_map_point;
				current_frame_->features_right_[i]->map_point_ = new_map_point;
				map_->InsertMapPoint(new_map_point);
				cnt_triangulated_pts++;
			}
		}
	}
	LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
	return (cnt_triangulated_pts);
}

// EstimateCurrentPose uses g2o optimization to refine the current camera pose based on
// feature observations. It constructs a pose vertex, creates projection edges from associated
// map points, runs several iterations of nonlinear optimization, and marks high-error edges
// as outliers. The refined pose is then set for the current frame, and features with large
// reprojection errors have their map point associations removed.
int Frontend::EstimateCurrentPose()
{
	std::shared_ptr<MapPoint> mp = nullptr;
	Mat33					K;
	int						index;
	EdgeProjectionPoseOnly	*edge;
	const double			chi2_th = 5.991;
	int						cnt_outlier;
	EdgeProjectionPoseOnly	*e = nullptr;

	// setup g2o
	// sets up g2o by defining the block solver type (6 DOF for pose and 3 for landmarks)
	typedef g2o::BlockSolver_6_3 BlockSolverType;
	typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
	auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(solver);


	// A new vertex representing the camera pose is created and assigned an ID of 0.
	// The current frame's pose is set as the initial estimate for the vertex.
	// The vertex is added to the optimizer.
	VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
	vertex_pose->setId(0);
	vertex_pose->setEstimate(current_frame_->Pose());
	optimizer.addVertex(vertex_pose);
	// K
	K = camera_left_->K();


	// 	The function iterates over all features in the left image of the current frame.
	// For each feature that has an associated map point (using a weak pointer lock), an edge is created
	index = 1;
	std::vector<EdgeProjectionPoseOnly *> edges;
	std::vector<Feature::Ptr> features;
	for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
	{
		mp = current_frame_->features_left_[i]->map_point_.lock();
		if (mp)
		{
			// The edge is constructed using the 3D position of the map point and the intrinsic matrix.
			// It is assigned an ID and connected to the pose vertex.
			// The measurement (2D observation) for the edge is set from the feature’s pixel coordinates.
			// The information matrix is set as an identity matrix (indicating equal weighting in both dimensions).
			// A robust kernel (Huber) is attached to handle outlier
			features.push_back(current_frame_->features_left_[i]);
			edge = new EdgeProjectionPoseOnly(mp->pos_, K);
			edge->setId(index);
			edge->setVertex(0, vertex_pose);
			edge->setMeasurement(toVec2(current_frame_->features_left_[i]->position_.pt));
			edge->setInformation(Eigen::Matrix2d::Identity());
			edge->setRobustKernel(new g2o::RobustKernelHuber);
			edges.push_back(edge);
			optimizer.addEdge(edge);
			index++;
		}
	}

	// Each iteration resets the pose estimate of the pose vertex to the current frame's pose.
	// It then initializes and runs the optimization (10 iterations per outer loop).
	// After optimization, it checks each edge's chi-squared error:
	// If an edge’s chi2 value exceeds the threshold, the corresponding feature is marked 
	// as an outlier and its edge level is set to 1; the outlier count is incremented.
	// Otherwise, the feature is considered an inlier and the edge level is set to 0.
	// On the third iteration (iteration == 2), the robust kernel is removed to fine-tune the optimization.
	cnt_outlier = 0;
	for (int iteration = 0; iteration < 4; ++iteration)
	{
		vertex_pose->setEstimate(current_frame_->Pose());
		optimizer.initializeOptimization();
		optimizer.optimize(10);
		cnt_outlier = 0;
		// count the outliers
		for (size_t i = 0; i < edges.size(); ++i)
		{
			e = edges[i];
			if (features[i]->is_outlier_)
			{
				e->computeError();
			}
			if (e->chi2() > chi2_th)
			{
				features[i]->is_outlier_ = true;
				e->setLevel(1);
				cnt_outlier++;
			}
			else
			{
				features[i]->is_outlier_ = false;
				e->setLevel(0);
			};
			if (iteration == 2)
			{
				e->setRobustKernel(nullptr);
			}
		}
	}
	LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/" << features.size()
		- cnt_outlier;
	// Set pose and outlier
	current_frame_->SetPose(vertex_pose->estimate());
	LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();
	for (auto &feat : features)
	{
		if (feat->is_outlier_)
		{
			feat->map_point_.reset();
			feat->is_outlier_ = false; // maybe we can still use it in future
		}
	}
	return (features.size() - cnt_outlier);
}




int Frontend::TrackLastFrame()
{
	std::shared_ptr<MapPoint> mp = nullptr;
	Vec2 px;
	Mat		error;
	int		num_good_pts;

	// use LK flow to estimate points in the right image
	std::vector<cv::Point2f> kps_last, kps_current;
	for (auto &kp : last_frame_->features_left_)
	{
		if (kp->map_point_.lock())
		{
			// use project point
			mp = kp->map_point_.lock();
			px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
			kps_last.push_back(kp->position_.pt);
			kps_current.push_back(cv::Point2f(px[0], px[1]));
		}
		else
		{
			kps_last.push_back(kp->position_.pt);
			kps_current.push_back(kp->position_.pt);
		}
	}
	std::vector<uchar> status;
	cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_,
		kps_last, kps_current, status, error, cv::Size(11, 11), 3,
	cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
			0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
	num_good_pts = 0;
	for (size_t i = 0; i < status.size(); ++i)
	{
		if (status[i])
		{
			cv::KeyPoint kp(kps_current[i], 7);
			Feature::Ptr feature(new Feature(current_frame_, kp));
			feature->map_point_ = last_frame_->features_left_[i]->map_point_;
			current_frame_->features_left_.push_back(feature);
			num_good_pts++;
		}
	}
	LOG(INFO) << "Find " << num_good_pts << " in the last image.";
	return (num_good_pts);
}

bool Frontend::StereoInit()
{
	int		num_features_left;
	int		num_coor_features;
	bool	build_map_success;

	num_features_left = DetectFeatures();
	num_coor_features = FindFeaturesInRight();
	if (num_coor_features < num_features_init_)
	{
		return (false);
	}
	build_map_success = BuildInitMap();
	if (build_map_success)
	{
		status_ = FrontendStatus::TRACKING_GOOD;
		if (viewer_)
		{
			viewer_->AddCurrentFrame(current_frame_);
			viewer_->UpdateMap();
		}
		return (true);
	}
	return (false);
}

int Frontend::DetectFeatures()
{
	int	cnt_detected;

	// Creating a Mask 
	cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
	for (auto &feat : current_frame_->features_left_)
	{
		cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
						feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
	}

	std::vector<cv::KeyPoint> keypoints;
	gftt_->detect(current_frame_->left_img_, keypoints, mask);
	cnt_detected = 0;

	// Creating a Feature object for each keypoint
	for (auto &kp : keypoints)
	{
		current_frame_->features_left_.push_back(Feature::Ptr(new Feature(current_frame_,kp)));
		cnt_detected++;
	}
	LOG(INFO) << "Detect " << cnt_detected << " new features";
	return (cnt_detected);
}

int Frontend::FindFeaturesInRight()
{
	std::shared_ptr<MapPoint> mp = nullptr;
	Vec2 px;
	Mat		error;
	int		num_good_pts;

	// use LK flow to estimate points in the right image
	std::vector<cv::Point2f> kps_left, kps_right;
	for (auto &kp : current_frame_->features_left_)
	{
		kps_left.push_back(kp->position_.pt);
		mp = kp->map_point_.lock();
		if (mp)
		{
			// use projected points as initial guess
			px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
			kps_right.push_back(cv::Point2f(px[0], px[1]));
		}
		else
		{
			// use same pixel in left iamge
			kps_right.push_back(kp->position_.pt);
		}
	}
	std::vector<uchar> status;
	cv::calcOpticalFlowPyrLK(current_frame_->left_img_,
		current_frame_->right_img_, kps_left, kps_right, status, error,
		cv::Size(11, 11), 3, cv::TermCriteria(cv::TermCriteria::COUNT
			+ cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
	num_good_pts = 0;
	for (size_t i = 0; i < status.size(); ++i)
	{
		if (status[i])
		{
			cv::KeyPoint kp(kps_right[i], 7);
			Feature::Ptr feat(new Feature(current_frame_, kp));
			feat->is_on_left_image_ = false;
			current_frame_->features_right_.push_back(feat);
			num_good_pts++;
		}
		else
		{
			current_frame_->features_right_.push_back(nullptr);
		}
	}
	LOG(INFO) << "Find " << num_good_pts << " in the right image.";
	return (num_good_pts);
}

bool Frontend::BuildInitMap()
{
	size_t	cnt_init_landmarks;
	Vec3	pworld;
	std::shared_ptr<MapPoint> new_map_point = nullptr;

	std::vector<SE3> poses{camera_left_->Pose(), camera_right_->Pose()};
	cnt_init_landmarks = 0;
	for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
	{
		if (current_frame_->features_right_[i] == nullptr)
			continue ;
		// create map point from triangulation
		std::vector<Vec3> points{camera_left_->pixel2camera(Vec2(current_frame_->features_left_[i]->position_.pt.x,
				current_frame_->features_left_[i]->position_.pt.y)),
			camera_right_->pixel2camera(Vec2(current_frame_->features_right_[i]->position_.pt.x,
				current_frame_->features_right_[i]->position_.pt.y))};
		pworld = Vec3::Zero();
		if (triangulation(poses, points, pworld) && pworld[2] > 0)
		{
			new_map_point = MapPoint::CreateNewMappoint();
			new_map_point->SetPos(pworld);
			new_map_point->AddObservation(current_frame_->features_left_[i]);
			new_map_point->AddObservation(current_frame_->features_right_[i]);
			current_frame_->features_left_[i]->map_point_ = new_map_point;
			current_frame_->features_right_[i]->map_point_ = new_map_point;
			cnt_init_landmarks++;
			map_->InsertMapPoint(new_map_point);
		}
	}
	current_frame_->SetKeyFrame();
	map_->InsertKeyFrame(current_frame_);
	backend_->UpdateMap();
	LOG(INFO) << "Initial map created with " << cnt_init_landmarks << " map points";
	return (true);
}

bool Frontend::Reset()
{
	LOG(INFO) << "Reset is not implemented. ";
	return (true);
}

} // namespace myslam