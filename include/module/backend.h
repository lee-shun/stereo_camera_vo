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

#include <Eigen/Core>

#include <memory>

namespace stereo_camera_vo {
namespace module {
class Backend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Backend> Ptr;

  void UpdateMap();
};
}  // namespace module
}  // namespace stereo_camera_vo

#endif  // INCLUDE_MODULE_BACKEND_H_
