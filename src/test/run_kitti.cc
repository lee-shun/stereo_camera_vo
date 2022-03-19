/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: run_kitti.cc
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

int main(int argc, char **argv) {
  std::string config_file = "./config/kitti.yaml";
  stereo_camera_vo::app::VisualOdometry::Ptr vo(
      new stereo_camera_vo::app::VisualOdometry(config_file));
  vo->Run();

  return 0;
}
