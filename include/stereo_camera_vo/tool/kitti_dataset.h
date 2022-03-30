/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: kitti_dataset.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_TOOL_KITTI_DATASET_H_
#define INCLUDE_STEREO_CAMERA_VO_TOOL_KITTI_DATASET_H_

#include "stereo_camera_vo/tool/dataset_base.h"

#include <memory>
#include <string>
#include <vector>

namespace stereo_camera_vo {
namespace tool {
class KittiDataset : public DatasetBase {
 public:
  typedef std::shared_ptr<KittiDataset> Ptr;
  explicit KittiDataset(const std::string& dataset_path);

  bool Init() override;

  bool NextFrame(common::Frame::Ptr new_frame) override;
};
}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_TOOL_KITTI_DATASET_H_
