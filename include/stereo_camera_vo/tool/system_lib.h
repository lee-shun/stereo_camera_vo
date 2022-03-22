/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: system_lib.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-22
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_STEREO_CAMERA_VO_TOOL_SYSTEM_LIB_H_
#define INCLUDE_STEREO_CAMERA_VO_TOOL_SYSTEM_LIB_H_

#include "stereo_camera_vo/tool/print_ctrl_macro.h"

#include <string>
#include <yaml-cpp/yaml.h>

namespace stereo_camera_vo {
namespace tool {
template <typename T>
T GetParam(const YAML::Node& node, const std::string& var_key,
           const T& default_value) {
  T v;
  try {
    v = node[var_key].as<T>();
  } catch (std::exception e) {
    v = default_value;
    PRINT_WARN("cannot find key: %s, set as default_value", var_key.c_str());
  }
  return v;
}
}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_TOOL_SYSTEM_LIB_H_
