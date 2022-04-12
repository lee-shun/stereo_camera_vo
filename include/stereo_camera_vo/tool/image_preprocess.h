/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: image_preprocess.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_TOOL_IMAGE_PREPROCESS_H_
#define INCLUDE_STEREO_CAMERA_VO_TOOL_IMAGE_PREPROCESS_H_

#include <opencv2/opencv.hpp>

namespace stereo_camera_vo {
namespace tool {

cv::Mat AddContrastness(cv::Mat gray);

void GammaTransform(cv::Mat &gary);

}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_TOOL_IMAGE_PREPROCESS_H_
