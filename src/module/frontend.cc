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

#include "module/frontend.h"
#include "common/feature.h"
#include "tool/config.h"
#include "tool/algorithm.h"
#include "tool/print_ctrl_macro.h"

#include <opencv2/opencv.hpp>

namespace stereo_camera_vo {
namespace module {

Frontend::Frontend() {
  gftt_ = cv::GFTTDetector::create(tool::Config::Get<int>("num_features"), 0.01,
                                   20);
  num_features_init_ = tool::Config::Get<int>("num_features_init");
  num_features_ = tool::Config::Get<int>("num_features");
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

bool Frontend::StereoInit() {
  int num_features_left = DetectFeatures();
  int num_coor_features = FindFeaturesInRight();
  if (num_coor_features < num_features_init_) {
    PRINT_WARN("faied to init stereo, left features num: %d, right: %d",
               num_features_left, num_coor_features);
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

int Frontend::DetectFeatures() {
  // mask the old feature point position.
  cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
  for (auto &feat : current_frame_->features_left_) {
    cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                  feat->position_.pt + cv::Point2f(10, 10), 0,
                  cv::LineTypes::FILLED);
  }

  std::vector<cv::KeyPoint> keypoints;
  gftt_->detect(current_frame_->left_img_, keypoints, mask);

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
  std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};
  size_t cnt_init_landmarks = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_right_[i] == nullptr) continue;
    // create map point from triangulation
    std::vector<Eigen::Vector3d> points{
        camera_left_->pixel2camera(
            Eigen::Vector2d(current_frame_->features_left_[i]->position_.pt.x,
                            current_frame_->features_left_[i]->position_.pt.y)),
        camera_right_->pixel2camera(Eigen::Vector2d(
            current_frame_->features_right_[i]->position_.pt.x,
            current_frame_->features_right_[i]->position_.pt.y))};
    Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

    if (tool::Triangulation(poses, points, pworld) && pworld[2] > 0) {
      auto new_map_point = common::MapPoint::CreateNewMappoint();
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

  PRINT_INFO("initial mao created with %zu map points!", cnt_init_landmarks);

  return true;
}

}  // namespace module
}  // namespace stereo_camera_vo
