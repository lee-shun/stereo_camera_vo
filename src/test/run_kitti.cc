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
#include "stereo_camera_vo/tool/kitti_dataset.h"

int main(int argc, char **argv) {
  const std::string config_file = "./config/frontend_config.yaml";
  const std::string dataset_path =
      "/media/ls/WORK/slam_kitti/dataset/sequences/15";

  stereo_camera_vo::tool::DatasetBase::Ptr dataset =
      std::make_shared<stereo_camera_vo::tool::KittiDataset>(dataset_path);

  stereo_camera_vo::app::VisualOdometry::Ptr vo(
      new stereo_camera_vo::app::VisualOdometry(config_file, dataset));

  vo->Run();

  return 0;
}
