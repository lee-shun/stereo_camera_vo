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

#include "common/map.h"
#include "tool/print_ctrl_macro.h"
#include "common/feature.h"

namespace stereo_camera_vo {
namespace common {
void Map::InsertKeyFrame(Frame::Ptr frame) {
  current_frame_ = frame;
  if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
    keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
  } else {
    keyframes_[frame->keyframe_id_] = frame;
    active_keyframes_[frame->keyframe_id_] = frame;
  }
  if (active_keyframes_.size() > num_active_keyframes_) {
    RemoveOldKeyframe();
  }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
  if (landmarks_.find(map_point->id_) == landmarks_.end()) {
    landmarks_.insert(make_pair(map_point->id_, map_point));
    active_landmarks_.insert(make_pair(map_point->id_, map_point));
  } else {
    landmarks_[map_point->id_] = map_point;
    active_landmarks_[map_point->id_] = map_point;
  }
}

void Map::RemoveOldKeyframe() {
  if (current_frame_ == nullptr) return;

  // find out the nearest and farest keyframe, use the distance
  double max_dis = 0, min_dis = 9999;
  double max_kf_id = 0, min_kf_id = 0;
  auto Twc = current_frame_->Pose().inverse();
  for (auto& kf : active_keyframes_) {
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
    // if there exists a nearest frame, remove it first.
    frame_to_remove = keyframes_.at(min_kf_id);
  } else {
    frame_to_remove = keyframes_.at(max_kf_id);
  }

  PRINT_INFO("remove keyframe %ld", frame_to_remove->keyframe_id_);

  // remove keyframe and landmark observation
  active_keyframes_.erase(frame_to_remove->keyframe_id_);

  for (auto feat : frame_to_remove->features_left_) {
    auto mp = feat->map_point_.lock();
    if (mp) {
      mp->RemoveObservation(feat);
    }
  }
  for (auto feat : frame_to_remove->features_right_) {
    if (feat == nullptr) continue;
    auto mp = feat->map_point_.lock();
    if (mp) {
      mp->RemoveObservation(feat);
    }
  }

  // after remove the old keyframes, landmarks may change.
  CleanMap();
}

void Map::CleanMap() {
  int cnt_landmark_removed = 0;
  for (auto iter = active_landmarks_.begin();
       iter != active_landmarks_.end();) {
    if (iter->second->observed_times_ == 0) {
      iter = active_landmarks_.erase(iter);
      cnt_landmark_removed++;
    } else {
      ++iter;
    }
  }
  PRINT_INFO("removed %d no-observed landmarks", cnt_landmark_removed);
}
}  // namespace common
}  // namespace stereo_camera_vo
