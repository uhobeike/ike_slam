// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__MCL_HPP_
#define IKE_SLAM__MCL_HPP_

#include "ike_slam/mapping/mapping.hpp"
#include "ike_slam/pf/likelihoodField.hpp"
#include "ike_slam/pf/motionModel.hpp"
#include "ike_slam/pf/observationModel.hpp"
#include "ike_slam/pf/particle.hpp"
#include "ike_slam/pf/resampling.hpp"
#include "ike_slam/pf/scan.hpp"
#include "ike_slam/scan_matching/scan_matching.hpp"

#include <memory>
#include <vector>

namespace mcl {
class Mcl {
public:
  Mcl(double ini_pose_x, double ini_pose_y, double ini_pose_yaw,
      double alpha_trans_trans, double alpha_trans_rotate,
      double alpha_rotate_trans, double alpha_rotate_rotate, int particle_size,
      double likelihood_dist, float map_resolution,
      bool publish_particles_scan_match_point);
  ~Mcl();

  void release_pointers();

  std::unique_ptr<mapping::Mapping> mapping_; // マッピングオブジェクト
  std::unique_ptr<scan_matching::ScanMatching>
      scan_matching_; // スキャンマッチングオブジェクト
  std::unique_ptr<MotionModel> motion_model_; // 動作モデルオブジェクト
  std::unique_ptr<ObservationModel>
      observation_model_;                  // 観測モデルオブジェクト
  std::unique_ptr<Resampling> resampling_; // リサンプリングオブジェクト

  void initParticles(double ini_pose_x, double ini_pose_y, double ini_pose_yaw,
                     int particle_size, double likelihood_dist,
                     float map_resolution); // パーティクルの初期化をする
  void getMeanParticle(Particle &particle); // 最尤なパーティクルを渡す
  inline float getMarginalLikelihood() {
    return observation_model_->marginal_likelihood_;
  } // 周辺尤度を渡す
  inline std::vector<std::pair<double, double>> getParticlesScanMatchPoint() {
    return observation_model_->particles_scan_match_point_;
  } // 各パーティクルのスキャンと尤度場のマッチポイントを渡す

  std::vector<Particle> particles_;
  Scan scan_;
};
} // namespace mcl

#endif
