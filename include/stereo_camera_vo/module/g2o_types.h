/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: g2o_types.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-13
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_STEREO_CAMERA_VO_MODULE_G2O_TYPES_H_
#define INCLUDE_STEREO_CAMERA_VO_MODULE_G2O_TYPES_H_

#include <Eigen/Core>

#include <sophus/se3.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>

namespace stereo_camera_vo {
namespace module {

/**
 * pose vertex
 * */
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void setToOriginImpl() override { _estimate = Sophus::SE3d(); }

  // left multiplication on Sophus::SE3d
  void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4],
        update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  bool read(std::istream &in) override { return true; }

  bool write(std::ostream &out) const override { return true; }
};

/**
 * landmarks vertex
 * */
class VertexXYZ : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  void setToOriginImpl() override { _estimate = Eigen::Vector3d::Zero(); }

  void oplusImpl(const double *update) override {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
  }

  bool read(std::istream &in) override { return true; }

  bool write(std::ostream &out) const override { return true; }
};

/**
 * unary edge for pose
 * */
class EdgeProjectionPoseOnly
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeProjectionPoseOnly(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K)
      : _pos3d(pos), _K(K) {}

  void computeError() override {
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }

  void linearizeOplus() override {
    const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_cam = T * _pos3d;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
        -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2,
        fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;
  }

  bool read(std::istream &in) override { return true; }

  bool write(std::ostream &out) const override { return true; }

 private:
  Eigen::Vector3d _pos3d;
  Eigen::Matrix3d _K;
};

/**
 * unary edge for pose and landmarks
 * */
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexXYZ> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // intrinsic and extrinsic param
  EdgeProjection(const Eigen::Matrix3d &K, const Sophus::SE3d &cam_ext)
      : _K(K) {
    _cam_ext = cam_ext;
  }

  void computeError() override {
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Eigen::Vector3d pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }

  void linearizeOplus() override {
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Eigen::Vector3d pw = v1->estimate();
    Eigen::Vector3d pos_cam = _cam_ext * T * pw;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
        -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y * Zinv2,
        fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;

    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                       _cam_ext.rotationMatrix() * T.rotationMatrix();
  }

  bool read(std::istream &in) override { return true; }

  bool write(std::ostream &out) const override { return true; }

 private:
  Eigen::Matrix3d _K;
  Sophus::SE3d _cam_ext;
};
}  // namespace module
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_MODULE_G2O_TYPES_H_
