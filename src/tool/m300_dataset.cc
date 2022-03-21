/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: m300_dataset.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-21
 *
 *   @Description:
 *
 *******************************************************************************/

#include "stereo_camera_vo/tool/m300_dataset.h"
#include "stereo_camera_vo/tool/print_ctrl_macro.h"

#include <iostream>

#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace stereo_camera_vo {
namespace tool {
bool M300Dataset::Init() {
  std::string camera_param = "./config/m300_front_stereo_param.yaml";

  file_.open(camera_param, cv::FileStorage::READ);
  if (!file_.isOpened()) {
    PRINT_ERROR("file %s is not opened", camera_param.c_str());
    file_.release();
    return false;
  }
  cv::Mat param_proj_left = getParameter<cv::Mat>("leftProjectionMatrix");
  cv::Mat param_proj_right = getParameter<cv::Mat>("rightProjectionMatrix");

  Eigen::Matrix3d left_K, right_K;
  Eigen::Vector3d left_t, right_t;
  convert2Eigen(param_proj_left, &left_K, &left_t);
  convert2Eigen(param_proj_right, &right_K, &right_t);

  // left
  left_t = Eigen::Vector3d::Zero();

  std::cout << "left K:\n" << left_K << std::endl;
  std::cout << "left t:\n" << left_t << std::endl;
  std::cout << "right K:\n" << right_K << std::endl;
  std::cout << "right t:\n" << right_t << std::endl;

  // Create new cameras
  common::Camera::Ptr left_cam = std::make_shared<common::Camera>(
      left_K(0, 0), left_K(1, 1), left_K(0, 2), left_K(1, 2), left_t.norm(),
      Sophus::SE3d(Sophus::SO3d(), left_t));
  cameras_.push_back(left_cam);

  common::Camera::Ptr right_cam = std::make_shared<common::Camera>(
      right_K(0, 0), right_K(1, 1), right_K(0, 2), right_K(1, 2),
      right_t.norm(), Sophus::SE3d(Sophus::SO3d(), right_t));
  cameras_.push_back(right_cam);

  return true;
}

void M300Dataset::convert2Eigen(const cv::Mat proj, Eigen::Matrix3d* K,
                                Eigen::Vector3d* t) {
  (*K) << proj.at<double>(0, 0), proj.at<double>(0, 1), proj.at<double>(0, 2),
      proj.at<double>(1, 0), proj.at<double>(1, 1), proj.at<double>(1, 2),
      proj.at<double>(2, 0), proj.at<double>(2, 1), proj.at<double>(2, 2);

  (*t) << proj.at<double>(0, 3), proj.at<double>(1, 3), proj.at<double>(2, 3);

  (*t) = (*K).inverse() * (*t);
}

common::Frame::Ptr M300Dataset::NextFrame() {
  boost::format fmt("%s/image_%d/%d.png");
  cv::Mat image_left, image_right;

  current_image_index_++;

  // read images
  image_left =
      cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                 cv::IMREAD_GRAYSCALE);
  image_right =
      cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                 cv::IMREAD_GRAYSCALE);

  if (image_left.data == nullptr || image_right.data == nullptr) {
    PRINT_WARN("can not find images at index %d", current_image_index_);
    return nullptr;
  }

  // cv::Mat image_left_resized, image_right_resized;
  // cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
  //            cv::INTER_NEAREST);
  // cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
  //            cv::INTER_NEAREST);

  auto new_frame = common::Frame::CreateFrame();
  new_frame->left_img_ = image_left;
  new_frame->right_img_ = image_right;
  return new_frame;
}

}  // namespace tool
}  // namespace stereo_camera_vo
