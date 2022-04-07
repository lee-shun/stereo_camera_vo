/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: map.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-10
 *
 *   @Description:
 *
 *******************************************************************************/

#include "stereo_camera_vo/common/map.h"
#include "stereo_camera_vo/tool/print_ctrl_macro.h"
#include "stereo_camera_vo/common/feature.h"

namespace stereo_camera_vo {
namespace common {
void Map::InsertKeyFrame(Frame::Ptr frame) {
  std::unique_lock<std::mutex> lck(data_mutex_);
  current_frame_ = frame;
  if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
    keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
  } else {
    keyframes_[frame->keyframe_id_] = frame;
    active_keyframes_[frame->keyframe_id_] = frame;
  }

  if (active_keyframes_.size() > num_active_keyframes_) {
    RemoveOldKeyframe(active_keyframes_);
    CleanLandmarks(active_landmarks_);
  }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
  std::unique_lock<std::mutex> lck(data_mutex_);
  if (landmarks_.find(map_point->id_) == landmarks_.end()) {
    landmarks_.insert(make_pair(map_point->id_, map_point));
    active_landmarks_.insert(make_pair(map_point->id_, map_point));
  } else {
    landmarks_[map_point->id_] = map_point;
    active_landmarks_[map_point->id_] = map_point;
  }
}

void Map::RemoveOldKeyframe(KeyframesType& keyframes) {
  if (current_frame_ == nullptr) return;

  // find out the nearest and farest keyframe, use the distance
  double max_dis = 0, min_dis = 9999;
  double max_kf_id = 0, min_kf_id = 0;
  auto Twc = current_frame_->Pose().inverse();

  for (auto& kf : keyframes) {
    if (kf.second == current_frame_) continue;
    auto dis = (kf.second->Pose() * Twc).log().norm();
    if (dis > max_dis) {
      max_dis = dis;
      max_kf_id = kf.first;
    }
    if (dis < min_dis) {
      min_dis = dis;
      min_kf_id = kf.first;
    }
  }

  const double min_dis_th = 0.2;
  Frame::Ptr frame_to_remove = nullptr;
  if (min_dis < min_dis_th) {
    // if there exists very near frame, remove it first.
    frame_to_remove = keyframes.at(min_kf_id);
  } else {
    // remove the farest frame.
    frame_to_remove = keyframes.at(max_kf_id);
  }

  PRINT_INFO("remove keyframe %ld", frame_to_remove->keyframe_id_);
  keyframes.erase(frame_to_remove->keyframe_id_);

  // landmark observation (features)
  for (auto feat : frame_to_remove->GetFeaturesLeft()) {
    auto map_point = feat->map_point_.lock();
    if (map_point) {
      map_point->RemoveObservation(feat);
    }
  }
  for (auto feat : frame_to_remove->GetFeaturesRight()) {
    if (feat == nullptr) continue;
    auto map_point = feat->map_point_.lock();
    if (map_point) {
      map_point->RemoveObservation(feat);
    }
  }
}

void Map::CleanLandmarks(LandmarksType& landmarks) {
  int cnt_landmark_removed = 0;
  for (auto iter = landmarks.begin(); iter != landmarks.end();) {
    if (iter->second->observed_times_ == 0) {
      iter = landmarks.erase(iter);
      cnt_landmark_removed++;
    } else {
      ++iter;
    }
  }
  PRINT_INFO("removed %d no-observed landmarks", cnt_landmark_removed);
}
}  // namespace common
}  // namespace stereo_camera_vo
