/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: algorithm.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_TOOL_ALGORITHM_H_
#define INCLUDE_STEREO_CAMERA_VO_TOOL_ALGORITHM_H_

#include <Eigen/Core>

#include <vector>

#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>

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
                          Eigen::Vector3d &pt_world);

// converters
inline Eigen::Vector2d ToVec2(const cv::Point2f p) {
  return Eigen::Vector2d(p.x, p.y);
}

}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_TOOL_ALGORITHM_H_
