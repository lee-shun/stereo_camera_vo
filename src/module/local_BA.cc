/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: local_BA.cc
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

#include "stereo_camera_vo/module/local_BA.h"
#include "stereo_camera_vo/tool/algorithm.h"
#include "stereo_camera_vo/tool/print_ctrl_macro.h"

#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>

namespace stereo_camera_vo {
namespace module {

// TODO(lee-shun): this local BA has memory leak problems???

LocalBA::LocalBA() {
  local_BA_running_.store(true);
  local_BA_thread_ = std::thread(std::bind(&LocalBA::ThreadLoop, this));
}

void LocalBA::UpdateMap() {
  std::unique_lock<std::mutex> lock(data_mutex_);
  map_update_.notify_one();
}

void LocalBA::Stop() {
  local_BA_running_.store(false);
  map_update_.notify_one();

  // release camera and map
  cam_left_ = nullptr;
  cam_right_ = nullptr;
  map_ = nullptr;

  PRINT_INFO("stop current local BA! wait for join!");
  local_BA_thread_.join();
}

void LocalBA::Detach() {
  local_BA_running_.store(false);
  map_update_.notify_one();

  // release camera and map
  cam_left_ = nullptr;
  cam_right_ = nullptr;
  map_ = nullptr;

  PRINT_INFO("detach current local BA!");
  local_BA_thread_.detach();
}

void LocalBA::ThreadLoop() {
  while (local_BA_running_.load()) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.wait(lock);

    // local BA optimize active ONLY
    if (nullptr != map_) {
      common::Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
      common::Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
      Optimize(active_kfs, active_landmarks);
    }
  }
}

void LocalBA::Optimize(common::Map::KeyframesType &keyframes,
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
   * Step: camera pose vertices bind with keyframe_id
   * */

  std::map<uint64_t, VertexPose *> veretx_pose_set;
  uint64_t max_kf_id = 0;
  for (auto &keyframe : keyframes) {
    auto kf = keyframe.second;
    VertexPose *vertex_pose = new VertexPose();

    vertex_pose->setId(kf->keyframe_id_);
    vertex_pose->setEstimate(kf->Pose());
    optimizer.addVertex(vertex_pose);

    if (kf->keyframe_id_ > max_kf_id) {
      max_kf_id = kf->keyframe_id_;
    }

    veretx_pose_set.insert({kf->keyframe_id_, vertex_pose});
  }

  /**
   * Step: landmark vertex and edges (observation (features) for each landmark)
   * */

  // each edge is each observation == each feature.
  std::map<uint64_t, VertexXYZ *> vertex_ldmk_set;
  std::map<EdgeProjection *, common::Feature::Ptr> edges_and_features;

  Eigen::Matrix3d K = cam_left_->K();
  Sophus::SE3d left_ext = cam_left_->pose();
  Sophus::SE3d right_ext = cam_right_->pose();

  double chi2_th = 5.991;
  int obs_index = 1;

  for (auto &ldmk : landmarks) {
    if (ldmk.second->is_outlier_) continue;

    // PRINT_DEBUG("ldmk.first == ldmk.second->id_ ? : %d",
    //             (ldmk.first == ldmk.second->id_));
    uint64_t landmark_id = ldmk.second->id_;
    auto observations = ldmk.second->GetObs();

    // add position vertex for each landmark
    if (vertex_ldmk_set.find(landmark_id) == vertex_ldmk_set.end()) {
      VertexXYZ *v = new VertexXYZ;
      v->setEstimate(ldmk.second->Pos());
      v->setId(landmark_id + max_kf_id + 1);
      v->setMarginalized(true);
      vertex_ldmk_set.insert({landmark_id, v});
      optimizer.addVertex(v);
    }

    // add observations(edges) for each landmark and frame pose
    for (auto &obs : observations) {
      if (obs.lock() == nullptr) continue;
      auto feat = obs.lock();
      if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

      auto frame = feat->frame_.lock();
      if (veretx_pose_set.find(frame->keyframe_id_) == veretx_pose_set.end())
        continue;

      EdgeProjection *edge = nullptr;
      if (feat->is_on_left_image_) {
        edge = new EdgeProjection(K, left_ext);
      } else {
        edge = new EdgeProjection(K, right_ext);
      }
      edge->setId(obs_index);
      edge->setVertex(0, veretx_pose_set.at(frame->keyframe_id_));  // pose
      edge->setVertex(1, vertex_ldmk_set.at(landmark_id));  // landmark
      edge->setMeasurement(tool::ToVec2(feat->position_.pt));
      edge->setInformation(Eigen::Matrix2d::Identity());
      auto rk = new g2o::RobustKernelHuber();
      rk->setDelta(chi2_th);
      edge->setRobustKernel(rk);
      edges_and_features.insert({edge, feat});
      optimizer.addEdge(edge);
      obs_index++;
    }
  }

  // do optimization and eliminate the outliers
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  UpdateChiTh(edges_and_features, &chi2_th);

  for (auto &ef : edges_and_features) {
    if (ef.first->chi2() > chi2_th) {
      ef.second->is_outlier_ = true;
      // remove the observation
      if (nullptr != ef.second->map_point_.lock())
        ef.second->map_point_.lock()->RemoveObservation(ef.second);
    } else {
      ef.second->is_outlier_ = false;
    }
  }

  // Set pose and lanrmark position
  for (auto &v : veretx_pose_set) {
    keyframes.at(v.first)->SetPose(v.second->estimate());
  }
  for (auto &v : vertex_ldmk_set) {
    landmarks.at(v.first)->SetPos(v.second->estimate());
  }
}

void LocalBA::UpdateChiTh(
    const std::map<EdgeProjection *, common::Feature::Ptr> &edges_and_features,
    double *chi2_th) {
  int cnt_outlier = 0, cnt_inlier = 0;
  int iteration = 0;

  while (iteration < 5) {
    cnt_outlier = 0;
    cnt_inlier = 0;
    // determine if we want to adjust the outlier threshold
    for (auto &ef : edges_and_features) {
      if (ef.first->chi2() > *chi2_th) {
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
      (*chi2_th) *= 2;
      iteration++;
    }
  }

  PRINT_INFO("outlier/inlier in pose estimating: %d/%d", cnt_outlier,
             cnt_inlier);
}

}  // namespace module
}  // namespace stereo_camera_vo
