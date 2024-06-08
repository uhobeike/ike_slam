// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__MAPPING_HPP_
#define IKE_SLAM__MAPPING_HPP_

#include "ike_slam/pf/likelihoodField.hpp"
#include "ike_slam/pf/particle.hpp"
#include "ike_slam/pf/pose.hpp"
#include "ike_slam/pf/scan.hpp"

#include <memory>

namespace mcl {
class Mapping {
public:
  Mapping(double resolution);
  ~Mapping();

  void gridMapping(std::vector<Particle> &particles, const Scan &scan);
  std::tuple<std::vector<std::pair<double, double>>,
             std::vector<std::pair<double, double>>>
  getUpdateCells(const Pose pose, const Scan &scan);
  void upadateCells(std::shared_ptr<LikelihoodField> likelihood_field,
                    std::vector<std::pair<double, double>> &scan_hit_cells,
                    std::vector<std::pair<double, double>> &scan_pass_cells);
  std::vector<std::pair<double, double>> calcScanHitCells(const Pose pose,
                                                          const Scan &scan);
  std::vector<std::pair<double, double>> calcScanPassCells(
      const Pose pose,
      const std::vector<std::pair<double, double>> &scan_hits_cells);

  std::vector<std::pair<double, double>> bresenham(int x0, int y0, int x1,
                                                   int y1);

  double resolution_;
};
} // namespace mcl

#endif