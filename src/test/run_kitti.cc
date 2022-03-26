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
#include "stereo_camera_vo/tool/system_lib.h"
#include "stereo_camera_vo/tool/kitti_dataset.h"

int main(int argc, char **argv) {
  YAML::Node node = YAML::LoadFile("./config/run_kitti.yaml");
  const uint64_t msleep_time =
      stereo_camera_vo::tool::GetParam(node, "msleep_time", 1000);

  const std::string config_file =
      stereo_camera_vo::tool::GetParam<std::string>(node, "config_file", "");

  const std::string dataset_path =
      stereo_camera_vo::tool::GetParam<std::string>(node, "dataset_path", "");
  stereo_camera_vo::tool::DatasetBase::Ptr dataset =
      std::make_shared<stereo_camera_vo::tool::KittiDataset>(dataset_path);

  stereo_camera_vo::app::VisualOdometry::Ptr vo(
      new stereo_camera_vo::app::VisualOdometry(config_file, dataset));

  vo->Run(msleep_time);

  return 0;
}
