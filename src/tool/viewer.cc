/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: viewer.cc
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

#include "common/feature.h"
#include "tool/viewer.h"
#include "tool/print_ctrl_macro.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sophus/se3.hpp>

#include <unistd.h>

namespace stereo_camera_vo {
namespace tool {

Viewer::Viewer() {
  viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Stop() {
  viewer_running_ = false;
  PRINT_INFO("stop current viewer! wait for join!");
  viewer_thread_.join();
}

void Viewer::AddCurrentFrame(common::Frame::Ptr current_frame) {
  std::unique_lock<std::mutex> lck(viewer_data_mutex_);
  current_frame_ = current_frame;
}

void Viewer::UpdateMap() {
  std::unique_lock<std::mutex> lck(viewer_data_mutex_);
  assert(map_ != nullptr);
  active_keyframes_ = map_->GetActiveKeyFrames();
  active_landmarks_ = map_->GetActiveMapPoints();
  map_updated_ = true;
}

void Viewer::ThreadLoop() {
  pangolin::CreateWindowAndBind("stereo_camera_vo_viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState vis_camera(
      pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& vis_display =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(vis_camera));

  const float green[3] = {0, 1, 0};
  while (!pangolin::ShouldQuit() && viewer_running_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    vis_display.Activate(vis_camera);

    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    if (nullptr != current_frame_) {
      DrawFrame(current_frame_, green);
      FollowCurrentFrame(vis_camera);

      cv::Mat img = PlotFrameImage();
      cv::imshow("image", img);
      cv::waitKey(1);
    }

    if (nullptr != map_) {
      DrawMapPoints();
    }

    pangolin::FinishFrame();
    usleep(5000);
  }
  PRINT_INFO("stop viewer");
}

cv::Mat Viewer::PlotFrameImage() {
  cv::Mat img_out;
  cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);

  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_left_[i]->map_point_.lock()) {
      auto feat = current_frame_->features_left_[i];
      cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0), 2);
    }
  }
  return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
  Sophus::SE3d Twc = current_frame_->Pose().inverse();
  pangolin::OpenGlMatrix m(Twc.matrix());
  vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(common::Frame::Ptr frame, const float* color) {
  Sophus::SE3d Twc = frame->Pose().inverse();
  const float sz = 1.0;
  const int line_width = 2.0;
  const float fx = 400;
  const float fy = 400;
  const float cx = 512;
  const float cy = 384;
  const float width = 1080;
  const float height = 768;

  glPushMatrix();

  Sophus::Matrix4f m = Twc.matrix().template cast<float>();
  glMultMatrixf(static_cast<GLfloat*>(m.data()));

  if (nullptr == color) {
    glColor3f(1, 0, 0);
  } else {
    glColor3f(color[0], color[1], color[2]);
  }

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glEnd();
  glPopMatrix();
}

void Viewer::DrawMapPoints() {
  const float red[3] = {1.0, 0, 0};
  for (auto& kf : active_keyframes_) {
    DrawFrame(kf.second, red);
  }

  glPointSize(2);
  glBegin(GL_POINTS);
  for (auto& landmark : active_landmarks_) {
    auto pos = landmark.second->Pos();
    glColor3f(red[0], red[1], red[2]);
    glVertex3d(pos[0], pos[1], pos[2]);
  }
  glEnd();
}
}  // namespace tool
}  // namespace stereo_camera_vo
