/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: visual_odometry.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-14
 *
 *   @Description:
 *
 *******************************************************************************/

#include "stereo_camera_vo/app/visual_odometry.h"
#include "stereo_camera_vo/tool/print_ctrl_macro.h"
#include "stereo_camera_vo/tool/system_lib.h"

#include <chrono>
#include <system_error>
#include <thread>

#include <unistd.h>

namespace stereo_camera_vo {
namespace app {

VisualOdometry::VisualOdometry(std::string frontend_config_path,
                               tool::DatasetBase::Ptr dataset)
    : dataset_(dataset) {
  if (!dataset_->Init()) {
    return;
  }

  // read vo parameters from config file
  module::Frontend::Param frontend_param;

  YAML::Node node = YAML::LoadFile(frontend_config_path);
  frontend_param.num_features_ = tool::GetParam<int>(node, "num_features", 200);
  frontend_param.num_features_init_ =
      tool::GetParam<int>(node, "num_features_init", 100);
  frontend_param.num_features_tracking_ =
      tool::GetParam<int>(node, "num_features_tracking", 50);
  frontend_param.num_features_tracking_bad_ =
      tool::GetParam<int>(node, "num_features_tracking_bad", 40);
  frontend_param.num_features_needed_for_keyframe_ =
      tool::GetParam<int>(node, "num_features_needed_for_keyframe", 80);

  // create frontend
  frontend_ = module::Frontend::Ptr(new module::Frontend(
      dataset_->GetCamera(0), dataset_->GetCamera(1), frontend_param, true));
}

void VisualOdometry::Run(const uint64_t msleep) {
  while (1) {
    PRINT_INFO("VO is running!");
    Step();
    std::this_thread::sleep_for(std::chrono::milliseconds(msleep));
  }
  frontend_->Stop();
  PRINT_INFO("VO exit!");
}

bool VisualOdometry::Step() {
  common::Frame::Ptr new_frame = common::Frame::CreateFrame();
  if (!dataset_->NextFrame(new_frame)) {
    PRINT_INFO("end of vo!");
    return false;
  }

  auto t1 = std::chrono::steady_clock::now();
  std::cout << "pose before: \n"
            << new_frame->Pose().inverse().matrix() << std::endl;
  bool success = frontend_->AddFrame(new_frame);
  std::cout << "pose after: \n"
            << new_frame->Pose().inverse().matrix() << std::endl;
  auto t2 = std::chrono::steady_clock::now();
  auto time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  PRINT_INFO("VO cost time: %lf", time_used.count());
  return success;
}
} /* namespace app */
}  // namespace stereo_camera_vo
