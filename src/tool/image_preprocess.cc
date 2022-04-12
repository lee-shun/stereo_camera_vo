/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: image_preprocess.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-04-12
 *
 *   @Description:
 *
 *******************************************************************************/

#include "stereo_camera_vo/tool/image_preprocess.h"

namespace stereo_camera_vo {
namespace tool {

cv::Mat AddContrastness(cv::Mat gray) {
  cv::Mat equa;
  cv::equalizeHist(gray, equa);
  return equa;
}
}  // namespace tool
}  // namespace stereo_camera_vo
