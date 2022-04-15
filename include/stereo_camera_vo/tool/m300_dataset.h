/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: m300_dataset.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_TOOL_M300_DATASET_H_
#define INCLUDE_STEREO_CAMERA_VO_TOOL_M300_DATASET_H_

#include "stereo_camera_vo/tool/dataset_base.h"

#include <Eigen/Core>

#include <string>
#include <fstream>

namespace stereo_camera_vo {
namespace tool {
class M300Dataset : public DatasetBase {
 public:
  explicit M300Dataset(const std::string dataset_path,
                       const uint16_t start_from = 1)
      : DatasetBase(dataset_path, start_from) {}

  bool Init() override;

  bool NextFrame(common::Frame::Ptr new_frame) override;

  static bool GetAttByIndex(const std::string pose_path,
                            const uint16_t pose_index, Eigen::Quaterniond* att);

 private:
  bool first_frame{true};
  // read the camera data
  cv::FileStorage camera_config_file_;

  // the first frame pose
  Sophus::SE3d first_frame_pose_Tcw_;

  template <typename T>
  T getParameter(const std::string key) {
    T t;
    camera_config_file_[key] >> t;
    return t;
  }

  void convert2Eigen(const cv::Mat proj, Eigen::Matrix3d* K,
                     Eigen::Vector3d* t);

  Sophus::SE3d Twb2Twc(const Sophus::SE3d& Twb) const;
};
}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_TOOL_M300_DATASET_H_
