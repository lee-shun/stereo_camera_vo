/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: viewer.h
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

#ifndef INCLUDE_STEREO_CAMERA_VO_TOOL_VIEWER_H_
#define INCLUDE_STEREO_CAMERA_VO_TOOL_VIEWER_H_

#include "stereo_camera_vo/common/map.h"
#include "stereo_camera_vo/common/frame.h"

#include <memory>
#include <thread>
#include <unordered_map>
#include <atomic>

#include <Eigen/Core>
#include <pangolin/pangolin.h>

namespace stereo_camera_vo {
namespace tool {
class Viewer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Viewer> Ptr;

  Viewer();

  void SetMap(common::Map::Ptr map) { map_ = map; }

  void Stop();

  void AddCurrentFrame(common::Frame::Ptr current_frame);

  void UpdateMap();

 private:
  void ThreadLoop();

  void DrawFrame(common::Frame::Ptr frame, const float* color);

  void DrawMap();

  void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

  /// plot the features in current frame into an image
  cv::Mat PlotFrameImage();

  common::Frame::Ptr current_frame_{nullptr};
  common::Map::Ptr map_{nullptr};

  std::thread viewer_thread_;
  std::atomic<bool> viewer_running_;

  std::unordered_map<uint64_t, common::Frame::Ptr> keyframes_;
  std::unordered_map<uint64_t, common::MapPoint::Ptr> landmarks_;

  std::unordered_map<uint64_t, common::Frame::Ptr> active_keyframes_;
  std::unordered_map<uint64_t, common::MapPoint::Ptr> active_landmarks_;

  std::mutex viewer_data_mutex_;
};
}  // namespace tool
}  // namespace stereo_camera_vo

#endif  // INCLUDE_STEREO_CAMERA_VO_TOOL_VIEWER_H_
