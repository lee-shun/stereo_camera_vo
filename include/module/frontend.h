/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: frontend.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-12
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_MODULE_FRONTEND_H_
#define INCLUDE_MODULE_FRONTEND_H_

#include "common/frame.h"
#include "common/map.h"
#include "common/camera.h"
#include "tool/viewer.h"
#include "module/backend.h"

#include <Eigen/Core>

#include <memory>

#include <sophus/se3.hpp>
#include <opencv2/features2d.hpp>

namespace stereo_camera_vo {
namespace module {

enum FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

class Frontend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<Frontend> Ptr;

  Frontend();

  bool AddFrame(common::Frame::Ptr frame);

  void SetMap(common::Map::Ptr map) { map_ = map; }

  void SetBackend(std::shared_ptr<module::Backend> backend) {
    backend_ = backend;
  }

  void SetViewer(std::shared_ptr<tool::Viewer> viewer) { viewer_ = viewer; }

  void SetCameras(common::Camera::Ptr left, common::Camera::Ptr right) {
    camera_left_ = left;
    camera_right_ = right;
  }

  FrontendStatus GetStatus() const { return status_; }

 private:
  /**
   * Track in normal mode
   * @return true if success
   */
  bool Track();

  /**
   * Reset when lost
   * @return true if success
   */
  bool Reset();

  /**
   * Track with last frame
   * @return num of tracked points
   */
  int TrackLastFrame();

  /**
   * estimate current frame's pose
   * @return num of inliers
   */
  int EstimateCurrentPose();

  /**
   * set current frame as a keyframe and insert it into backend
   * @return true if success
   */
  bool InsertKeyframe();

  /**
   * Try init the frontend with stereo images saved in current_frame_
   * @return true if success
   */
  bool StereoInit();

  /**
   * Detect new features in left image in current_frame_
   * keypoints will be saved in current_frame_
   * @return the number of new features
   */
  int DetectFeatures();

  /**
   * Find the corresponding features in right image of current_frame_
   * @return num of features found
   */
  int FindFeaturesInRight();

  /**
   * Build the initial map with single image
   * @return true if succeed
   */
  bool BuildInitMap();

  /**
   * Triangulate the 2D points in current frame
   * @return num of triangulated points
   */
  int TriangulateNewPoints();

  /**
   * Set the features in keyframe as new observation of the map points
   */
  void SetObservationsForKeyFrame();

  FrontendStatus status_{FrontendStatus::INITING};

  common::Frame::Ptr current_frame_{nullptr};
  common::Frame::Ptr last_frame_{nullptr};
  common::Camera::Ptr camera_left_{nullptr};
  common::Camera::Ptr camera_right_{nullptr};

  common::Map::Ptr map_{nullptr};
  std::shared_ptr<module::Backend> backend_{nullptr};
  std::shared_ptr<tool::Viewer> viewer_{nullptr};

  Sophus::SE3d relative_motion_;

  uint16_t tracking_inliers_{0};  // inliers, used for testing new keyframes

  // params
  int num_features_{200};
  int num_features_init_{100};
  int num_features_tracking_{50};
  int num_features_tracking_bad_{20};
  int num_features_needed_for_keyframe_{80};

  // utilities
  cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
};
}  // namespace module
}  // namespace stereo_camera_vo

#endif  // INCLUDE_MODULE_FRONTEND_H_
