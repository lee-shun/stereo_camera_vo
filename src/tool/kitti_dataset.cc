/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: kitti_dataset.cc
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

#include "stereo_camera_vo/tool/kitti_dataset.h"
#include "stereo_camera_vo/tool/print_ctrl_macro.h"

#include <fstream>

#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.hpp>

namespace stereo_camera_vo {
namespace tool {

KittiDataset::KittiDataset(const std::string& dataset_path)
    : DatasetBase(dataset_path, 0) {}

bool KittiDataset::Init() {
  std::ifstream fin(dataset_path_ + "/calib.txt");
  if (!fin) {
    PRINT_ERROR("can not open %s in given path, no such file or directory!",
                (dataset_path_ + "/calib.txt").c_str());
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    char camera_name[3];
    for (int k = 0; k < 3; ++k) {
      fin >> camera_name[k];
    }
    double projection_data[12];
    for (int k = 0; k < 12; ++k) {
      fin >> projection_data[k];
    }
    Eigen::Matrix3d K;
    K << projection_data[0], projection_data[1], projection_data[2],
        projection_data[4], projection_data[5], projection_data[6],
        projection_data[8], projection_data[9], projection_data[10];
    Eigen::Vector3d t;
    t << projection_data[3], projection_data[7], projection_data[11];
    t = K.inverse() * t;
    // K = K * 0.5;

    common::Camera::Ptr new_camera(
        new common::Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(),
                           Sophus::SE3d(Sophus::SO3d(), t)));
    cameras_.push_back(new_camera);
  }

  fin.close();
  current_image_index_ = 0;
  return true;
}

bool KittiDataset::NextFrame(common::Frame::Ptr new_frame) {
  boost::format fmt("%s/image_%d/%06d.png");
  cv::Mat image_left, image_right;
  // read images
  image_left =
      cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                 cv::IMREAD_GRAYSCALE);
  image_right =
      cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                 cv::IMREAD_GRAYSCALE);

  if (image_left.data == nullptr || image_right.data == nullptr) {
    PRINT_ERROR("cannot find images at index %d!", current_image_index_);
    return false;
  }

  cv::Mat image_left_resized, image_right_resized;
  cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
             cv::INTER_NEAREST);
  cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
             cv::INTER_NEAREST);

  new_frame->left_img_ = image_left;
  new_frame->right_img_ = image_right;
  current_image_index_++;

  return true;
}

}  // namespace tool
}  // namespace stereo_camera_vo
