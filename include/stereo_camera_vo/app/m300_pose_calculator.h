/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: m300_pose_calculator.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-20
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_STEREO_CAMERA_VO_APP_M300_POSE_CALCULATOR_H_
#define INCLUDE_STEREO_CAMERA_VO_APP_M300_POSE_CALCULATOR_H_

#include "stereo_camera_vo/common/camera.h"
#include "stereo_camera_vo/common/frame.h"
#include "stereo_camera_vo/module/frontend.h"

#include "stereo_camera_vo/tool/print_ctrl_macro.h"

#include <memory>
#include <csignal>
#include <string>

#include <sophus/se3.hpp>

namespace stereo_camera_vo {
namespace app {
class M300PoseCalculator {
 public:
  M300PoseCalculator();

  void Step();

 private:
  stereo_camera_vo::common::Camera::Ptr camera_left_{nullptr};
  stereo_camera_vo::common::Camera::Ptr camera_right_{nullptr};
  stereo_camera_vo::module::Frontend::Ptr frontend_{nullptr};
};
}  // namespace app
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_APP_M300_POSE_CALCULATOR_H_
