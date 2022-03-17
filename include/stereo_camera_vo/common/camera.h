/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: camera.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-11
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_STEREO_CAMERA_VO_COMMON_CAMERA_H_
#define INCLUDE_STEREO_CAMERA_VO_COMMON_CAMERA_H_

#include <Eigen/Core>

#include <memory>

#include <sophus/se3.hpp>

namespace stereo_camera_vo {
namespace common {
struct Camera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Camera> Ptr;

  Camera();

  Camera(double fx, double fy, double cx, double cy, double baseline,
         const Sophus::SE3d &pose)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
    pose_inv_ = pose_.inverse();
  }

  Sophus::SE3d pose() const { return pose_; }

  // return intrinsic matrix
  Eigen::Matrix3d K() const {
    Eigen::Matrix3d k;
    k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    return k;
  }

  // coordinate transform: world, camera, pixel
  Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w,
                               const Sophus::SE3d &T_c_w);

  Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c,
                               const Sophus::SE3d &T_c_w);

  Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);

  Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1);

  Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p,
                              const Sophus::SE3d &T_c_w, double depth = 1);

  Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w,
                              const Sophus::SE3d &T_c_w);

 public:
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
         baseline_ = 0;    // Camera intrinsics
  Sophus::SE3d pose_;      // extrinsic, from stereo camera to single camera
  Sophus::SE3d pose_inv_;  // inverse of extrinsics
};
}  // namespace common
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_COMMON_CAMERA_H_
