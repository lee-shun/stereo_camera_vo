/*******************************************************************************
*   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
*
*   @Filename: run_m300.cc
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


#include "stereo_camera_vo/app/visual_odometry.h"
#include "stereo_camera_vo/tool/m300_dataset.h"

int main(int argc, char **argv) {
  const std::string config_file = "./config/vo_config.yaml";
  const std::string dataset_path =
      "/media/ls/WORK/slam_m300/m300_data";

  stereo_camera_vo::tool::DatasetBase::Ptr dataset =
      std::make_shared<stereo_camera_vo::tool::M300Dataset>(dataset_path);

  stereo_camera_vo::app::VisualOdometry::Ptr vo(
      new stereo_camera_vo::app::VisualOdometry(config_file, dataset));

  vo->Run();

  return 0;
}
