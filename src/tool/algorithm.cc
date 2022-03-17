/*******************************************************************************
*   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
*
*   @Filename: algorithm.cc
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


#include "stereo_camera_vo/tool/algorithm.h"

namespace stereo_camera_vo {
namespace tool {

/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
bool Triangulation(const std::vector<Sophus::SE3d> &poses,
                          const std::vector<Eigen::Vector3d> points,
                          Eigen::Vector3d &pt_world) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(2 * poses.size(), 4);
  Eigen::Matrix<double, Eigen::Dynamic, 1> b(2 * poses.size());
  b.setZero();
  for (size_t i = 0; i < poses.size(); ++i) {
    Eigen::Matrix<double, 3, 4> m = poses[i].matrix3x4();
    A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
    A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
  }
  auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

  if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
    return true;
  }

  // bad solution
  return false;
}

}  // namespace tool
}  // namespace stereo_camera_vo
