/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: feature.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_COMMON_FEATURE_H_
#define INCLUDE_STEREO_CAMERA_VO_COMMON_FEATURE_H_

// h
#include <Eigen/Core>

// std cpp
#include <memory>

// hpp
#include <opencv2/core.hpp>

namespace stereo_camera_vo {
namespace common {

struct Frame;
struct MapPoint;

struct Feature {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Feature> Ptr;

  Feature() {}

  Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
      : frame_(frame), position_(kp) {}

 public:
  std::weak_ptr<Frame> frame_;
  cv::KeyPoint position_;
  std::weak_ptr<MapPoint> map_point_;

  bool is_outlier_{false};
  bool is_on_left_image_{true};
};
}  // namespace common
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_COMMON_FEATURE_H_
