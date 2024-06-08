// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_slam/pf/likelihoodField.hpp"
#include <cmath>
#include <vector>

namespace mcl {
LikelihoodField::LikelihoodField(double likelihood_dist, float resolution)
    : likelihood_dist_(likelihood_dist), resolution_(resolution),
      likelihood_dist_sq_(likelihood_dist * likelihood_dist) {
  std::cerr << "Create LikelihoodField.\n";
  std::cerr << "Done Create LikelihoodField.\n";
};

LikelihoodField::~LikelihoodField(){};

void LikelihoodField::createLikelihoodField(int map_x, int map_y) {
  double max_prob = calculateProb(0, likelihood_dist_);
  double sigma = likelihood_dist_ / 3;
  double normalizer = 1.0 / std::sqrt(2.0 * M_PI * sigma * sigma);

  for (int y = map_y - likelihood_dist_; y <= map_y + likelihood_dist_; y++) {
    for (int x = map_x - likelihood_dist_; x <= map_x + likelihood_dist_; x++) {
      double dx = x - map_x;
      double dy = y - map_y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < likelihood_dist_sq_) {
        double dist = std::sqrt(dist_sq);
        double exp_val = std::exp(-dist_sq / (2 * sigma * sigma));
        double calculated_prob = normalizer * exp_val;
        double normalized_prob = normalizePdf(max_prob, calculated_prob);
        if (normalized_prob > smap_.getValueFromCell<double>(x, y, true)) {
          smap_.addCell(x, y, normalized_prob);
        }
      }
    }
  }
};

inline double LikelihoodField::calculateProb(double distance,
                                             double likelihood_dist) {
  double sigma = likelihood_dist / 3;
  double x = -distance * distance / (2 * sigma * sigma);
  return 1.0 / std::sqrt(2.0 * M_PI * sigma * sigma) * std::exp(x);
}

inline double LikelihoodField::normalizePdf(double max_pdf, double pdf) {
  return (pdf / max_pdf) * 100;
}
} // namespace mcl
