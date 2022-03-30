/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_m300_dataset.cc
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
#include "stereo_camera_vo/tool/system_lib.h"

#include <chrono>
#include <iostream>

int main(int argc, char** argv) {
  YAML::Node node = YAML::LoadFile("./config/run_m300.yaml");
  const std::string dataset_path =
      stereo_camera_vo::tool::GetParam<std::string>(node, "dataset_path", "");

  stereo_camera_vo::tool::DatasetBase::Ptr dataset =
      std::make_shared<stereo_camera_vo::tool::M300Dataset>(dataset_path);

  dataset->Init();

  std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();

  int num = 0;
  stereo_camera_vo::common::Frame::Ptr new_frame =
      stereo_camera_vo::common::Frame::CreateFrame();
  while (dataset->NextFrame(new_frame)) {
    PRINT_INFO("----------------");
    std::cout << "pose: \n" << new_frame->Pose().matrix() << std::endl;
  stereo_camera_vo::common::Frame::Ptr new_frame =
      stereo_camera_vo::common::Frame::CreateFrame();
    ++num;
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  double each_t =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count() /
      static_cast<double>(num);
  PRINT_INFO("average: %lf ms", each_t);

  return 0;
}
