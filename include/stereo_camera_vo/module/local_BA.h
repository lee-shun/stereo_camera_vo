/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: local_BA.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_MODULE_LOCAL_BA_H_
#define INCLUDE_STEREO_CAMERA_VO_MODULE_LOCAL_BA_H_

#include "stereo_camera_vo/common/camera.h"
#include "stereo_camera_vo/common/map.h"
#include "stereo_camera_vo/module/g2o_types.h"
#include "stereo_camera_vo/common/feature.h"

#include <Eigen/Core>

#include <memory>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <map>

namespace stereo_camera_vo {
namespace module {
class LocalBA {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<LocalBA> Ptr;

  /**
   * creat local BA thread and hangon
   * */
  LocalBA();

  ~LocalBA() { Detach(); }

  void SetCameras(common::Camera::Ptr left, common::Camera::Ptr right) {
    cam_left_ = left;
    cam_right_ = right;
  }

  void SetMap(std::shared_ptr<common::Map> map) { map_ = map; }

  /**
   * toggle one times of local BA optimization
   * */
  void UpdateMap();

  void Restart();
  void Detach();

  void Stop();

 private:
  void ThreadLoop();

  void UpdateChiTh(
      const std::map<EdgeProjection*, common::Feature::Ptr>& edges_and_features,
      double* chi2_th);

  void Optimize(common::Map::KeyframesType& keyframes,
                common::Map::LandmarksType& landmarks);

  std::shared_ptr<common::Map> map_;
  std::thread local_BA_thread_;
  std::mutex data_mutex_;

  std::condition_variable map_update_;
  std::atomic<bool> local_BA_running_;

  common::Camera::Ptr cam_left_{nullptr}, cam_right_{nullptr};
};
}  // namespace module
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_MODULE_LOCAL_BA_H_
