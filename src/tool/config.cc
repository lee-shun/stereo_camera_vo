/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: config.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-11
 *
 *   @Description:
 *
 *******************************************************************************/

#include "tool/config.h"
#include "tool/print_ctrl_macro.h"

namespace stereo_camera_vo {
namespace tool {

bool Config::SetParameterFile(const std::string &filename) {
  if (config_ == nullptr) config_ = std::shared_ptr<Config>(new Config);
  config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);

  if (config_->file_.isOpened() == false) {
    PRINT_ERROR("parameter file: %s does NOT exist!", filename.c_str());
    config_->file_.release();
    return false;
  }

  return true;
}

Config::~Config() {
  if (file_.isOpened()) file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;

}  // namespace tool
}  // namespace stereo_camera_vo
