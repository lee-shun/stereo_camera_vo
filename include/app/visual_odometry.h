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

#ifndef INCLUDE_APP_VISUAL_ODOMETRY_H_
#define INCLUDE_APP_VISUAL_ODOMETRY_H_

#include "module/frontend.h"
#include "module/backend.h"
#include "tool/dataset.h"
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
  explicit VisualOdometry(std::string &config_path);

  /**
   * do initialization things before run
   * @return true if success
   */
  bool Init();

  /**
   * start vo in the dataset
   */
  void Run();

  /**
   * Make a step forward in dataset
   */
  bool Step();

  module::FrontendStatus GetFrontendStatus() const {
    return frontend_->GetStatus();
  }

 private:
  bool inited_{false};
  std::string config_file_path_;

  module::Frontend::Ptr frontend_{nullptr};
  module::Backend::Ptr backend_{nullptr};
  common::Map::Ptr map_{nullptr};
  tool::Viewer::Ptr viewer_{nullptr};

  // dataset
  tool::Dataset::Ptr dataset_{nullptr};
};
}  // namespace app
}  // namespace stereo_camera_vo

#endif  // INCLUDE_APP_VISUAL_ODOMETRY_H_
