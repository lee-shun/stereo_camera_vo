/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: map_point.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-09
 *
 *   @Description:
 *
 *******************************************************************************/

#include "stereo_camera_vo/common/map_point.h"
#include "stereo_camera_vo/common/feature.h"

namespace stereo_camera_vo {
namespace common {

MapPoint::MapPoint(uint64_t id, Eigen::Vector3d position)
    : id_(id), pos_(position) {}

MapPoint::Ptr MapPoint::CreateNewMappoint() {
  static uint64_t factory_id;
  MapPoint::Ptr new_mappoint(new MapPoint);
  new_mappoint->id_ = factory_id++;
  return new_mappoint;
}

/**
 * current map point observed_times will decrease
 * */
void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
  std::unique_lock<std::mutex> lck(data_mutex_);
  for (auto iter = observations_.begin(); iter != observations_.end(); iter++) {
    if (iter->lock() == feat) {
      observations_.erase(iter);
      feat->map_point_.reset();
      observed_times_--;
      break;
    }
  }
}
}  // namespace common

}  // namespace stereo_camera_vo
