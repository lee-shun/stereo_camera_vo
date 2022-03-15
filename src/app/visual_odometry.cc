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

#include "app/visual_odometry.h"
#include "tool/config.h"
#include "tool/print_ctrl_macro.h"

#include <unistd.h>

namespace stereo_camera_vo {
namespace app {

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

bool VisualOdometry::Init() {
  // read from config file
  if (tool::Config::SetParameterFile(config_file_path_) == false) {
    return false;
  }

  dataset_ = tool::Dataset::Ptr(
      new tool::Dataset(tool::Config::Get<std::string>("dataset_dir")));
  if (!dataset_->Init()) {
    return false;
  }

  // create components and links
  frontend_ = module::Frontend::Ptr(new module::Frontend);
  backend_ = module::Backend::Ptr(new module::Backend);
  map_ = common::Map::Ptr(new common::Map);
  viewer_ = tool::Viewer::Ptr(new tool::Viewer);

  frontend_->SetBackend(backend_);
  frontend_->SetMap(map_);
  frontend_->SetViewer(viewer_);
  frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

  backend_->SetMap(map_);
  backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

  viewer_->SetMap(map_);

  return true;
}

void VisualOdometry::Run() {
  while (1) {
    PRINT_INFO("VO is running!");
    if (Step() == false) {
      break;
    }
    // sleep(5);
  }

  backend_->Stop();
  viewer_->Close();

  PRINT_INFO("VO exit!");
}

bool VisualOdometry::Step() {
  common::Frame::Ptr new_frame = dataset_->NextFrame();
  if (new_frame == nullptr) return false;

  auto t1 = std::chrono::steady_clock::now();
  bool success = frontend_->AddFrame(new_frame);
  auto t2 = std::chrono::steady_clock::now();
  auto time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  PRINT_INFO("VO cost time: %lf", time_used.count());
  return success;
}
} /* namespace app */
}  // namespace stereo_camera_vo
