/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: backend.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-03-13
 *
 *   @Description:
 *
 *******************************************************************************/

#include "module/backend.h"
#include "common/feature.h"
#include "module/g2o_types.h"
#include "tool/algorithm.h"
#include "tool/print_ctrl_macro.h"

#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>

#include <map>

namespace stereo_camera_vo {
namespace module {

// TODO(lee-shun): this backend has memory leak problems???

Backend::Backend() {
  backend_running_.store(true);
  backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap() {
  std::unique_lock<std::mutex> lock(data_mutex_);
  map_update_.notify_one();
}

void Backend::Stop() {
  backend_running_.store(false);
  map_update_.notify_one();
  backend_thread_.join();
}

void Backend::BackendLoop() {
  while (backend_running_.load()) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.wait(lock);

    // backend optimize active ONLY
    common::Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
    common::Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
    Optimize(active_kfs, active_landmarks);
  }
}

void Backend::Optimize(common::Map::KeyframesType &keyframes,
                       common::Map::LandmarksType &landmarks) {
  // setup g2o
  typedef g2o::BlockSolver_6_3 BlockSolverType;
  typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  /**
   * camera pose bind with keyframe_id
   * */
  std::map<uint64_t, VertexPose *> vertices;
  uint64_t max_kf_id = 0;
  for (auto &keyframe : keyframes) {
    auto kf = keyframe.second;
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(kf->keyframe_id_);
    vertex_pose->setEstimate(kf->Pose());
    optimizer.addVertex(vertex_pose);
    if (kf->keyframe_id_ > max_kf_id) {
      max_kf_id = kf->keyframe_id_;
    }

    vertices.insert({kf->keyframe_id_, vertex_pose});
  }

  std::map<uint64_t, VertexXYZ *> vertices_landmarks;

  Eigen::Matrix3d K = cam_left_->K();
  Sophus::SE3d left_ext = cam_left_->pose();
  Sophus::SE3d right_ext = cam_right_->pose();

  int index = 1;
  double chi2_th = 5.991;
  // each edge is each observation == each feature.
  std::map<EdgeProjection *, common::Feature::Ptr> edges_and_features;

  for (auto &landmark : landmarks) {
    if (landmark.second->is_outlier_) continue;

    uint64_t landmark_id = landmark.second->id_;
    auto observations = landmark.second->GetObs();

    /**
     * each observation to a landmark generates an edge
     * */
    for (auto &obs : observations) {
      if (obs.lock() == nullptr) continue;
      auto feat = obs.lock();
      if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

      EdgeProjection *edge = nullptr;
      if (feat->is_on_left_image_) {
        edge = new EdgeProjection(K, left_ext);
      } else {
        edge = new EdgeProjection(K, right_ext);
      }

      // add xyz vertices for each landmark
      if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end()) {
        VertexXYZ *v = new VertexXYZ;
        v->setEstimate(landmark.second->Pos());
        v->setId(landmark_id + max_kf_id + 1);
        v->setMarginalized(true);
        vertices_landmarks.insert({landmark_id, v});
        optimizer.addVertex(v);
      }

      edge->setId(index);
      auto frame = feat->frame_.lock();
      edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
      edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
      edge->setMeasurement(tool::ToVec2(feat->position_.pt));
      edge->setInformation(Eigen::Matrix2d::Identity());
      auto rk = new g2o::RobustKernelHuber();
      rk->setDelta(chi2_th);
      edge->setRobustKernel(rk);

      edges_and_features.insert({edge, feat});

      optimizer.addEdge(edge);

      index++;
    }
  }

  // do optimization and eliminate the outliers
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  int cnt_outlier = 0, cnt_inlier = 0;
  int iteration = 0;
  while (iteration < 5) {
    cnt_outlier = 0;
    cnt_inlier = 0;
    // determine if we want to adjust the outlier threshold
    for (auto &ef : edges_and_features) {
      if (ef.first->chi2() > chi2_th) {
        cnt_outlier++;
      } else {
        cnt_inlier++;
      }
    }
    double inlier_ratio =
        cnt_inlier / static_cast<double>(cnt_inlier + cnt_outlier);
    if (inlier_ratio > 0.5) {
      break;
    } else {
      chi2_th *= 2;
      iteration++;
    }
  }

  for (auto &ef : edges_and_features) {
    if (ef.first->chi2() > chi2_th) {
      ef.second->is_outlier_ = true;
      // remove the observation
      ef.second->map_point_.lock()->RemoveObservation(ef.second);
    } else {
      ef.second->is_outlier_ = false;
    }
  }

  PRINT_INFO("outlier/Inlier in pose estimating: %d/%d", cnt_outlier,
             cnt_inlier);

  // Set pose and lanrmark position
  for (auto &v : vertices) {
    keyframes.at(v.first)->SetPose(v.second->estimate());
  }
  for (auto &v : vertices_landmarks) {
    landmarks.at(v.first)->SetPos(v.second->estimate());
  }
}
}  // namespace module
}  // namespace stereo_camera_vo
