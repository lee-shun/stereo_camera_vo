/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: map_point.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_COMMON_MAP_POINT_H_
#define INCLUDE_STEREO_CAMERA_VO_COMMON_MAP_POINT_H_

// h
#include <Eigen/Core>

// std cpp
#include <mutex>
#include <list>
#include <memory>

// hpp
#include <opencv2/core.hpp>

namespace stereo_camera_vo {
namespace common {

struct Frame;
struct Feature;

struct MapPoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<MapPoint> Ptr;

  MapPoint() {}

  MapPoint(uint64_t id, Eigen::Vector3d position);

  Eigen::Vector3d Pos() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return pos_;
  }

  void SetPos(const Eigen::Vector3d &pos) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    pos_ = pos;
  }

  void AddObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    observations_.push_back(feature);
    observed_times_++;
  }

  void RemoveObservation(std::shared_ptr<Feature> feat);

  std::list<std::weak_ptr<Feature>> GetObs() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return observations_;
  }

  static MapPoint::Ptr CreateNewMappoint();

 public:
  uint64_t id_{0};
  bool is_outlier_{false};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  std::mutex data_mutex_;
  int observed_times_{0};
  std::list<std::weak_ptr<Feature>> observations_;
};
}  // namespace common
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_COMMON_MAP_POINT_H_
