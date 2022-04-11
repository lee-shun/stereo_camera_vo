/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: map.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-10
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_STEREO_CAMERA_VO_COMMON_MAP_H_
#define INCLUDE_STEREO_CAMERA_VO_COMMON_MAP_H_

#include "stereo_camera_vo/common/frame.h"
#include "stereo_camera_vo/common/map_point.h"
#include "stereo_camera_vo/tool/print_ctrl_macro.h"

#include <Eigen/Core>

#include <mutex>
#include <memory>
#include <unordered_map>

namespace stereo_camera_vo {
namespace common {
class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Map> Ptr;
  typedef std::unordered_map<uint64_t, MapPoint::Ptr> LandmarksType;
  typedef std::unordered_map<uint64_t, Frame::Ptr> KeyframesType;

  Map() {}

  ~Map() { PRINT_INFO("old map destroyed!"); }

  void InsertKeyFrame(Frame::Ptr frame);

  void InsertMapPoint(MapPoint::Ptr map_point);

  LandmarksType GetAllMapPoints() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return landmarks_;
  }

  KeyframesType GetAllKeyFrames() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return keyframes_;
  }

  LandmarksType GetActiveMapPoints() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return active_landmarks_;
  }

  KeyframesType GetActiveKeyFrames() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return active_keyframes_;
  }

 private:
  /**
   * Deactive the old KeyFrames
   * */
  void RemoveOldKeyframe(KeyframesType& keyframes);

  /**
   * clean the map point that observations is 0
   * */
  void CleanLandmarks(LandmarksType& landmarks);

  std::mutex data_mutex_;

  LandmarksType landmarks_;
  LandmarksType active_landmarks_;
  KeyframesType keyframes_;
  KeyframesType active_keyframes_;

  Frame::Ptr current_frame_{nullptr};

  const u_int16_t num_active_keyframes_{7};
};
}  // namespace common
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_COMMON_MAP_H_
