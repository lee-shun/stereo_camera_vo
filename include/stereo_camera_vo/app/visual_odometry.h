/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: visual_odometry.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-14
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_STEREO_CAMERA_VO_APP_VISUAL_ODOMETRY_H_
#define INCLUDE_STEREO_CAMERA_VO_APP_VISUAL_ODOMETRY_H_

#include "stereo_camera_vo/module/frontend.h"
#include "stereo_camera_vo/tool/dataset_base.h"
#include <Eigen/Core>

#include <memory>
#include <string>

namespace stereo_camera_vo {
namespace app {
class VisualOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<VisualOdometry> Ptr;

  /// constructor with config file
  explicit VisualOdometry(std::string frontend_config_path,
                          tool::DatasetBase::Ptr dataset);
  /**
   * start vo in the dataset
   */
  void Run(const uint64_t msleep = 0);

  /**
   * Make a step forward in dataset
   */
  bool Step();

  module::FrontendStatus GetFrontendStatus() const {
    return frontend_->GetStatus();
  }

 private:
  bool inited_{false};

  module::Frontend::Ptr frontend_{nullptr};

  // dataset
  tool::DatasetBase::Ptr dataset_{nullptr};
};
}  // namespace app
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_APP_VISUAL_ODOMETRY_H_
