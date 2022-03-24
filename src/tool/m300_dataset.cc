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
#include "stereo_camera_vo/tool/system_lib.h"

#include <iostream>

#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace stereo_camera_vo {
namespace tool {
bool M300Dataset::Init() {
  /**
   * Step: get cameras
   * */
  const std::string camera_param =
      dataset_path_ + "/m300_front_stereo_param.yaml";
  camera_config_file_.open(camera_param, cv::FileStorage::READ);
  if (!camera_config_file_.isOpened()) {
    PRINT_ERROR("file %s is not opened", camera_param.c_str());
    camera_config_file_.release();
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

  // Create new cameras
  common::Camera::Ptr left_cam = std::make_shared<common::Camera>(
      left_K(0, 0), left_K(1, 1), left_K(0, 2), left_K(1, 2), left_t.norm(),
      Sophus::SE3d(Sophus::SO3d(), left_t));
  cameras_.push_back(left_cam);

  common::Camera::Ptr right_cam = std::make_shared<common::Camera>(
      right_K(0, 0), right_K(1, 1), right_K(0, 2), right_K(1, 2),
      right_t.norm(), Sophus::SE3d(Sophus::SO3d(), right_t));
  cameras_.push_back(right_cam);

  /**
   * get pose
   * */
  const std::string pose_file = dataset_path_ + "/pose.txt";
  pose_fin_.open(pose_file);
  if (!pose_fin_) {
    PRINT_ERROR("can not open %s in given path, no such file or directory!",
                pose_file.c_str());
    return false;
  }

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
  PRINT_DEBUG(" current_image_index_: %d", current_image_index_);

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
  // create frame
  auto new_frame = common::Frame::CreateFrame();
  new_frame->left_img_ = image_left;
  new_frame->right_img_ = image_right;

  /*
   * Step: read pose
   * */
  std::string pose_tmp;
  std::vector<double> q_elements;

  tool::SeekToLine(pose_fin_, current_image_index_-1);
  // read each w, x, y, z, everytime
  for (int i = 0; i < 4; ++i) {
    if (!getline(pose_fin_, pose_tmp, ',')) {
      PRINT_WARN("pose reading error! at index %d", current_image_index_);
      return nullptr;
    }
    // PRINT_DEBUG("read pose-wxyz:%.8f", std::stod(pose_tmp));
    q_elements.push_back(std::stod(pose_tmp));
  }

  Eigen::Quaterniond q(q_elements[0], q_elements[1], q_elements[2],
                       q_elements[3]);

  Sophus::SE3d pose_body_Twb(q, Eigen::Vector3d::Zero());

  Sophus::SE3d pose_cam_Tcw = Twb2Twc(pose_body_Twb).inverse();

  if (0 == current_image_index_ - 1) {
    first_frame_pose_Tcw_ = pose_cam_Tcw;
    std::cout << "First frame:\n"
              << first_frame_pose_Tcw_.matrix() << std::endl;
  }

  // get current frame pose (it is actually relative motion according to the
  // first frame ...)
  Sophus::SE3d realtive_pose_Tcw =
      pose_cam_Tcw * first_frame_pose_Tcw_.inverse();

  new_frame->SetPose(realtive_pose_Tcw);

  return new_frame;
}

Sophus::SE3d M300Dataset::Twb2Twc(const Sophus::SE3d& Twb) const {
  Eigen::Quaterniond rotate_quat_bc;

  rotate_quat_bc.w() = 0.5f;
  rotate_quat_bc.x() = -0.5f;
  rotate_quat_bc.y() = 0.5f;
  rotate_quat_bc.z() = -0.5f;

  // camera and body coordinate only have a rotation between them...
  Sophus::SE3d Tbc(rotate_quat_bc, Eigen::Vector3d::Zero());

  return Twb * Tbc;
}

}  // namespace tool
}  // namespace stereo_camera_vo
