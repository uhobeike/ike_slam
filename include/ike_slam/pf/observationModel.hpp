// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__OBSERVATIONMODEL_HPP_
#define IKE_SLAM__OBSERVATIONMODEL_HPP_

#include "ike_slam/pf/likelihoodField.hpp"
#include "ike_slam/pf/particle.hpp"
#include "ike_slam/pf/scan.hpp"

#include <cmath>
#include <memory>

namespace mcl {
class ObservationModel {
public:
  ObservationModel(std::shared_ptr<mcl::LikelihoodField> likelihood_field,
                   bool publish_particles_scan_match_point);
  ~ObservationModel();

  void update(std::vector<Particle> &particles,
              const Scan &scan_data); // 観測モデルの更新
  double
  calculateParticleWeight(const Particle p,
                          const Scan &scan); // パーティクルの重みを計算する
  double getProbFromLikelihoodMap(double x,
                                  double y); // 尤度場から確率を取得する
  std::vector<double>
  getProbsFromLikelihoodMap(std::vector<std::pair<double, double>> &points);

  inline double getRadian(double degree) {
    return degree * M_PI / 180;
  } // ラジアンに変換する

  std::shared_ptr<mcl::LikelihoodField> likelihood_field_;

  float marginal_likelihood_;
  std::vector<std::pair<double, double>> particles_scan_match_point_;
  bool publish_particles_scan_match_point_;
};
} // namespace mcl

#endif