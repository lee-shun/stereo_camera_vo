/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: frame.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-09
 *
 *   @Description:
 *
 *******************************************************************************/

#include "stereo_camera_vo/common/frame.h"

namespace stereo_camera_vo {
namespace common {

Frame::Frame(uint64_t id, double time_stamp, const Sophus::SE3d &pose,
             const cv::Mat &left, const cv::Mat &right)
    : id_(id),
      time_stamp_(time_stamp),
      pose_(pose),
      left_img_(left),
      right_img_(right) {}

Frame::Ptr Frame::CreateFrame() {
  static uint64_t factory_id;
  Frame::Ptr new_frame(new Frame);
  new_frame->id_ = factory_id++;
  return new_frame;
}

void Frame::SetKeyFrame() {
  static uint64_t keyframe_factory_id;
  std::unique_lock<std::mutex> lck(data_mutex_);
  is_keyframe_ = true;
  keyframe_id_ = keyframe_factory_id++;
}

}  // namespace common
}  // namespace stereo_camera_vo
