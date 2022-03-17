/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: config.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_TOOL_CONFIG_H_
#define INCLUDE_STEREO_CAMERA_VO_TOOL_CONFIG_H_

#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace stereo_camera_vo {
namespace tool {
class Config {
 private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;

  Config() {}  // private constructor makes a singleton
 public:
  ~Config();  // close the file when deconstructing

  // set a new config file
  static bool SetParameterFile(const std::string &filename);

  // access the parameter values
  template <typename T>
  static T Get(const std::string &key) {
    return T(Config::config_->file_[key]);
  }
};

}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_TOOL_CONFIG_H_
