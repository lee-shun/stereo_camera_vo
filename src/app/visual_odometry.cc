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
#include "stereo_camera_vo/tool/config.h"
#include "stereo_camera_vo/tool/print_ctrl_macro.h"

#include <chrono>
#include <system_error>
#include <thread>

#include <unistd.h>

namespace stereo_camera_vo {
namespace app {

VisualOdometry::VisualOdometry(std::string config_path,
                               tool::DatasetBase::Ptr dataset)
    : dataset_(dataset) {
  // read vo parameters from config file
  if (tool::Config::SetParameterFile(config_path) == false) {
    return;
  }

  if (!dataset_->Init()) {
    return;
  }

  // create components and links
  frontend_ = module::Frontend::Ptr(new module::Frontend(
      dataset_->GetCamera(0), dataset_->GetCamera(1), config_path, true));
}

void VisualOdometry::Run(const uint64_t msleep) {
  while (1) {
    PRINT_INFO("VO is running!");
    if (Step() == false) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(msleep));
  }
  frontend_->Stop();
  PRINT_INFO("VO exit!");
}

bool VisualOdometry::Step() {
  common::Frame::Ptr new_frame = dataset_->NextFrame();
  if (new_frame == nullptr) return false;

  auto t1 = std::chrono::steady_clock::now();
  std::cout << "pose before: \n" << new_frame->Pose().matrix() << std::endl;
  bool success = frontend_->AddFrame(new_frame);
  std::cout << "pose after: \n" << new_frame->Pose().matrix() << std::endl;
  auto t2 = std::chrono::steady_clock::now();
  auto time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  PRINT_INFO("VO cost time: %lf", time_used.count());
  return success;
}
} /* namespace app */
}  // namespace stereo_camera_vo
