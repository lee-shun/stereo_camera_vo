/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: dataset_base.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_TOOL_DATASET_BASE_H_
#define INCLUDE_STEREO_CAMERA_VO_TOOL_DATASET_BASE_H_

#include "stereo_camera_vo/common/frame.h"
#include "stereo_camera_vo/common/camera.h"

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>

namespace stereo_camera_vo {
namespace tool {
class DatasetBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<DatasetBase> Ptr;

  explicit DatasetBase(const std::string dataset_path,
                       const uint16_t start_from_index)
      : dataset_path_(dataset_path), current_image_index_(start_from_index) {}

  virtual ~DatasetBase() {}

  virtual bool Init() = 0;

  virtual bool NextFrame(common::Frame::Ptr new_frame) = 0;

  common::Camera::Ptr GetCamera(const int camera_id) const {
    return cameras_.at(camera_id);
  }

  uint16_t GetIndex() const { return current_image_index_; }

 protected:
  std::string dataset_path_;
  uint16_t current_image_index_{0};

  std::vector<common::Camera::Ptr> cameras_;
};
}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_TOOL_DATASET_BASE_H_
