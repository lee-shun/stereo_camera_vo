/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: frame.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-09
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_STEREO_CAMERA_VO_COMMON_FRAME_H_
#define INCLUDE_STEREO_CAMERA_VO_COMMON_FRAME_H_

// h
#include <Eigen/Core>

// std cpp
#include <mutex>
#include <memory>
#include <vector>

// hpp
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>

namespace stereo_camera_vo {
namespace common {

struct MapPoint;
struct Feature;

struct Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<Frame> Ptr;

  Frame() {}

  Frame(uint64_t id, double time_stamp, const Sophus::SE3d &pose,
        const cv::Mat &left, const cv::Mat &right);

  // Note the pose is Tcw;
  Sophus::SE3d Pose() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return pose_;
  }

  // Note the pose is Tcw;
  void SetPose(const Sophus::SE3d &pose) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    pose_ = pose;
  }

  void SetKeyFrame();

  static std::shared_ptr<Frame> CreateFrame();

 public:
  uint64_t id_{0};
  uint64_t keyframe_id_{0};
  bool is_keyframe_{false};
  bool use_init_pose_{false};
  double time_stamp_;

  // Note the pose is Tcw;
  Sophus::SE3d pose_;
  std::mutex data_mutex_;

  cv::Mat left_img_, right_img_;

 public:
  std::vector<std::shared_ptr<Feature>>& GetFeaturesLeft() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return features_left_;
  }
  std::vector<std::shared_ptr<Feature>>& GetFeaturesRight() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return features_right_;
  }

 private:
  // extracted features in left image
  std::vector<std::shared_ptr<Feature>> features_left_;
  // corresponding features in right image, set to nullptr if no corresponding
  std::vector<std::shared_ptr<Feature>> features_right_;
};
}  // namespace common
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_COMMON_FRAME_H_
