/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: dataset.h
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

#ifndef INCLUDE_TOOL_DATASET_H_
#define INCLUDE_TOOL_DATASET_H_

#include "common/frame.h"
#include "common/camera.h"

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace stereo_camera_vo {
namespace tool {
class Dataset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Dataset> Ptr;
  explicit Dataset(const std::string& dataset_path);

  bool Init();

  common::Frame::Ptr NextFrame();

  common::Camera::Ptr GetCamera(int camera_id) const {
    return cameras_.at(camera_id);
  }

 private:
  std::string dataset_path_;
  int current_image_index_{0};

  std::vector<common::Camera::Ptr> cameras_;
};
}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_TOOL_DATASET_H_
