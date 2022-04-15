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

void GammaTransform(cv::Mat &gray) {
  const float gamma = log(128.0 / 255.0) / log(cv::mean(gray)[0] / 255.0);
  cv::Mat look_up_tab = cv::Mat::zeros(cv::Size(1, 256), CV_8UC1);
  for (int i = 0; i < 256; i++)
    look_up_tab.at<uchar>(0, i) = pow(i / 255.0, gamma) * 255.0;

  cv::LUT(gray, look_up_tab, gray);
}
}  // namespace tool

}  // namespace stereo_camera_vo
