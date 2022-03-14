/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: backend.h
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

#ifndef INCLUDE_MODULE_BACKEND_H_
#define INCLUDE_MODULE_BACKEND_H_

#include "common/camera.h"
#include "common/map.h"
#include <Eigen/Core>

#include <memory>
#include <thread>
#include <condition_variable>
#include <atomic>

namespace stereo_camera_vo {
namespace module {
class Backend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Backend> Ptr;

  /**
   * creat backend thread and hangon
   * */
  Backend();

  void SetCameras(common::Camera::Ptr left, common::Camera::Ptr right) {
    cam_left_ = left;
    cam_right_ = right;
  }

  void SetMap(std::shared_ptr<common::Map> map) { map_ = map; }

  /**
   * toggle one times of backend optimization
   * */
  void UpdateMap();

  void Stop();

 private:
  void BackendLoop();

  void Optimize(common::Map::KeyframesType& keyframes,
                common::Map::LandmarksType& landmarks);

  std::shared_ptr<common::Map> map_;
  std::thread backend_thread_;
  std::mutex data_mutex_;

  std::condition_variable map_update_;
  std::atomic<bool> backend_running_;

  common::Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};
}  // namespace module
}  // namespace stereo_camera_vo

#endif  // INCLUDE_MODULE_BACKEND_H_
