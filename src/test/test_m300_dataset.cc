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

int main(int argc, char** argv) {
  stereo_camera_vo::tool::M300Dataset m300_dataset(
      "./config/m300_front_stereo_param.yaml");
  m300_dataset.Init();
  return 0;
}
