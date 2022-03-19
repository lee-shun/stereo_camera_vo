/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: frontend.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-12
 *
 *   @Description:
 *
 *******************************************************************************/

#include "stereo_camera_vo/module/frontend.h"
#include "stereo_camera_vo/module/g2o_types.h"
#include "stereo_camera_vo/common/feature.h"
#include "stereo_camera_vo/tool/config.h"
#include "stereo_camera_vo/tool/algorithm.h"
#include "stereo_camera_vo/tool/print_ctrl_macro.h"

#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>

#include <opencv2/opencv.hpp>

namespace stereo_camera_vo {
namespace module {

Frontend::Frontend(common::Camera::Ptr left, common::Camera::Ptr right,
                   bool use_viewer, std::string config_file_path)
    : camera_left_(left), camera_right_(right) {
  /**
   * update parameters
   * */
  tool::Config::SetParameterFile(config_file_path);
  num_features_ = tool::Config::Get<int>("num_features");
  num_features_init_ = tool::Config::Get<int>("num_features_init");
  num_features_tracking_ = tool::Config::Get<int>("num_features_tracking");
  num_features_tracking_bad_ =
      tool::Config::Get<int>("num_features_tracking_bad");
  num_features_needed_for_keyframe_ =
      tool::Config::Get<int>("num_features_needed_for_keyframe");

  /**
   * init submodules, all smart pointers
   * */
  cv_detector_ = cv::GFTTDetector::create(num_features_, 0.01, 10);

  map_ = common::Map::Ptr(new common::Map);

  local_BA_ = module::LocalBA::Ptr(new module::LocalBA);
  local_BA_->SetMap(map_);
  local_BA_->SetCameras(camera_left_, camera_right_);

  if (use_viewer) {
    viewer_ = tool::Viewer::Ptr(new tool::Viewer);
    viewer_->SetMap(map_);
  }
}

bool Frontend::AddFrame(common::Frame::Ptr frame) {
  current_frame_ = frame;

  switch (status_) {
    case FrontendStatus::INITING:
      StereoInit();
      break;
    case FrontendStatus::TRACKING_GOOD:
    case FrontendStatus::TRACKING_BAD:
      Track();
      break;
    case FrontendStatus::LOST:
      Reset();
      break;
  }

  last_frame_ = current_frame_;
  return true;
}

bool Frontend::Track() {
  if (nullptr != last_frame_) {
    current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
  }

  TrackLastFrame();
  num_tracking_inliers_ = EstimateCurrentPose();

  if (num_tracking_inliers_ > num_features_tracking_) {
    status_ = FrontendStatus::TRACKING_GOOD;
  } else if (num_tracking_inliers_ > num_features_tracking_bad_) {
    status_ = FrontendStatus::TRACKING_BAD;
  } else {
    status_ = FrontendStatus::LOST;
  }

  UpdateMapWithFrame();

  if (nullptr != viewer_) {
    viewer_->AddCurrentFrame(current_frame_);
  }

  relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
  return true;
}

int Frontend::TrackLastFrame() {
  // calculate the initial guess
  std::vector<cv::Point2f> kps_last, kps_current;
  for (auto &kp : last_frame_->features_left_) {
    if (kp->map_point_.lock()) {
      // use project point
      auto mp = kp->map_point_.lock();
      auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
      kps_last.push_back(kp->position_.pt);
      kps_current.push_back(cv::Point2f(px[0], px[1]));
    } else {
      kps_last.push_back(kp->position_.pt);
      kps_current.push_back(kp->position_.pt);
    }
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
      last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current,
      status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_current[i], 7);
      common::Feature::Ptr feature(new common::Feature(current_frame_, kp));
      feature->map_point_ = last_frame_->features_left_[i]->map_point_;
      current_frame_->features_left_.push_back(feature);
      num_good_pts++;
    }
  }

  PRINT_INFO("find %d good tracked points with last image!", num_good_pts);
  return num_good_pts;
}

int Frontend::EstimateCurrentPose() {
  // setup g2o
  typedef g2o::BlockSolver_6_3 BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // vertex
  VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
  vertex_pose->setId(0);
  vertex_pose->setEstimate(current_frame_->Pose());
  optimizer.addVertex(vertex_pose);

  // K
  Eigen::Matrix3d K = camera_left_->K();

  // edges
  int index = 1;
  std::vector<EdgeProjectionPoseOnly *> edges;
  std::vector<common::Feature::Ptr> features;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    auto mp = current_frame_->features_left_[i]->map_point_.lock();
    if (mp) {
      features.push_back(current_frame_->features_left_[i]);
      EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
      edge->setId(index);
      edge->setVertex(0, vertex_pose);
      edge->setMeasurement(
          tool::ToVec2(current_frame_->features_left_[i]->position_.pt));
      edge->setInformation(Eigen::Matrix2d::Identity());
      edge->setRobustKernel(new g2o::RobustKernelHuber);
      edges.push_back(edge);
      optimizer.addEdge(edge);
      index++;
    }
  }

  // estimate the Pose the determine the outliers
  const double chi2_th = 5.991;
  int cnt_outlier = 0;
  for (int iteration = 0; iteration < 4; ++iteration) {
    vertex_pose->setEstimate(current_frame_->Pose());
    // setLevel(int) is useful when you call
    // optimizer.initializeOptimization(int). If you assign
    // initializeOptimization(0), the optimizer will include all edges up to
    // level 0 in the optimization, and edges set to level >=1 will not be
    // included.
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cnt_outlier = 0;

    // count the outliers
    for (size_t i = 0; i < edges.size(); ++i) {
      auto e = edges[i];
      if (features[i]->is_outlier_) {
        e->computeError();
      }
      if (e->chi2() > chi2_th) {
        features[i]->is_outlier_ = true;
        e->setLevel(1);
        cnt_outlier++;
      } else {
        features[i]->is_outlier_ = false;
        e->setLevel(0);
      }

      if (iteration == 2) {
        e->setRobustKernel(nullptr);
      }
    }
  }

  // Set pose and outlier
  current_frame_->SetPose(vertex_pose->estimate());
  for (auto &feat : features) {
    if (feat->is_outlier_) {
      feat->map_point_.reset();
      feat->is_outlier_ = false;  // maybe we can still use it in future
    }
  }

  PRINT_INFO("outlier/Inlier in pose estimating: %d/%lu", cnt_outlier,
             features.size() - cnt_outlier);
  return features.size() - cnt_outlier;
}

bool Frontend::UpdateMapWithFrame() {
  if (num_tracking_inliers_ >= num_features_needed_for_keyframe_) {
    // still have enough features, don't have potential to be a keyframe.
    return false;
  }
  // current frame is a new keyframe
  current_frame_->SetKeyFrame();
  PRINT_INFO("set frame id: %lu to keyframe keyframe_id: %lu",
             current_frame_->id_, current_frame_->keyframe_id_);

  SetObservationsForKeyFrame();

  // re-detect and add more map points for a keyframe if possible
  DetectNewFeatures();
  FindFeaturesInRight();
  TriangulateNewPoints();

  map_->InsertKeyFrame(current_frame_);
  local_BA_->UpdateMap();

  if (nullptr != viewer_) {
    viewer_->UpdateMap();
  }
  return true;
}

void Frontend::SetObservationsForKeyFrame() {
  for (auto &feat : current_frame_->features_left_) {
    auto mp = feat->map_point_.lock();
    if (mp) mp->AddObservation(feat);
  }
}

int Frontend::TriangulateNewPoints() {
  std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};
  Sophus::SE3d current_pose_Twc = current_frame_->Pose().inverse();
  int cnt_triangulated_pts = 0;

  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    PRINT_DEBUG("current_frame_->features_left_.size() = %zu",
                current_frame_->features_left_.size());

    // if features in left don't bind to a mappoint, at the meanwhile, right
    // image have corresponding features, Try to triangulate these new points.
    if (current_frame_->features_left_[i]->map_point_.expired() &&
        current_frame_->features_right_[i] != nullptr) {
      PRINT_DEBUG("current_frame_->features_left_[i]->map_point_.expired() =%d",
                  current_frame_->features_left_[i]->map_point_.expired());

      std::vector<Eigen::Vector3d> points{
          camera_left_->pixel2camera(Eigen::Vector2d(
              current_frame_->features_left_[i]->position_.pt.x,
              current_frame_->features_left_[i]->position_.pt.y)),
          camera_right_->pixel2camera(Eigen::Vector2d(
              current_frame_->features_right_[i]->position_.pt.x,
              current_frame_->features_right_[i]->position_.pt.y))};

      Eigen::Vector3d pworld = Eigen::Vector3d::Zero();
      PRINT_DEBUG("tool::Triangulation(poses, points, pworld) = %d",
                  tool::Triangulation(poses, points, pworld));
      if (tool::Triangulation(poses, points, pworld) && pworld[2] > 0) {
        PRINT_DEBUG("in the cnt++");
        auto new_map_point = common::MapPoint::CreateNewMappoint();
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
  PRINT_INFO("triangulate new landmarks: %d", cnt_triangulated_pts);
  return cnt_triangulated_pts;
}

bool Frontend::StereoInit() {
  int num_features_left = DetectNewFeatures();
  int num_features_right = FindFeaturesInRight();
  if (num_features_right < num_features_init_) {
    PRINT_WARN(
        "Due to inadequate features in right image, failed to init stereo, "
        "left features num: %d, right: %d (need %d)",
        num_features_left, num_features_right, num_features_init_);

    return false;
  }

  if (BuildInitMap()) {
    status_ = FrontendStatus::TRACKING_GOOD;
    if (nullptr != viewer_) {
      viewer_->AddCurrentFrame(current_frame_);
      viewer_->UpdateMap();
    }
    return true;
  }
  return false;
}

int Frontend::DetectNewFeatures() {
  // mask the old feature point position.
  cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
  for (auto &feat : current_frame_->features_left_) {
    cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                  feat->position_.pt + cv::Point2f(10, 10), 0,
                  cv::LineTypes::FILLED);
  }

  std::vector<cv::KeyPoint> keypoints;
  cv_detector_->detect(current_frame_->left_img_, keypoints, mask);

  int cnt_detected = 0;
  for (auto &kp : keypoints) {
    current_frame_->features_left_.push_back(
        common::Feature::Ptr(new common::Feature(current_frame_, kp)));
    cnt_detected++;
  }

  PRINT_INFO("detected %d new features in current image...", cnt_detected);
  return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
  // use LK flow to estimate points in the right image
  std::vector<cv::Point2f> kps_left, kps_right;

  // init the kps_left, and kps_right.
  for (auto &kp : current_frame_->features_left_) {
    kps_left.push_back(kp->position_.pt);
    auto mp = kp->map_point_.lock();
    if (mp) {
      // use projected points as initial guess
      auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
      kps_right.push_back(cv::Point2f(px[0], px[1]));
    } else {
      // use same pixel in left iamge
      kps_right.push_back(kp->position_.pt);
    }
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
      current_frame_->left_img_, current_frame_->right_img_, kps_left,
      kps_right, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (1 == status[i]) {
      cv::KeyPoint kp(kps_right[i], 7);
      common::Feature::Ptr feat(new common::Feature(current_frame_, kp));
      feat->is_on_left_image_ = false;
      current_frame_->features_right_.push_back(feat);
      num_good_pts++;
    } else {
      current_frame_->features_right_.push_back(nullptr);
    }
  }
  PRINT_INFO("find %d good feature points via LK flow in right image.",
             num_good_pts);
  return num_good_pts;
}

bool Frontend::BuildInitMap() {
  int cnt_init_landmarks = TriangulateNewPoints();

  current_frame_->SetKeyFrame();
  map_->InsertKeyFrame(current_frame_);
  local_BA_->UpdateMap();

  PRINT_INFO("initial map created with %d map points!", cnt_init_landmarks);

  return true;
}

bool Frontend::Reset() {
  PRINT_WARN("reset VO!");
  current_frame_ = nullptr;
  last_frame_ = nullptr;

  map_ = common::Map::Ptr(new common::Map);

  local_BA_->SetMap(map_);

  if (nullptr != viewer_) {
    viewer_->SetMap(map_);
    viewer_->AddCurrentFrame(current_frame_);
    viewer_->UpdateMap();
  }

  status_ = FrontendStatus::INITING;
  return true;
}
}  // namespace module
}  // namespace stereo_camera_vo
