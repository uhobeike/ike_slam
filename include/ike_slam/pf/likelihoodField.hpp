// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__LIKELIHOODFIELD_HPP_
#define IKE_SLAM__LIKELIHOODFIELD_HPP_

#include "ike_slam/map/sparseOccupancyGridMap.hpp"

#include <iostream>
#include <vector>

namespace mcl {

typedef Eigen::Vector2d Point;
typedef std::vector<Eigen::Vector2d> PointCloud;

class LikelihoodField {
public:
  LikelihoodField(double likelihood_dist, float resolution);
  ~LikelihoodField();

  void createLikelihoodField(int map_x,
                             int map_y); // 尤度場作成のための計算をする
  double
  calculateProb(double stochastic_variable,
                double likelihood_dist); // 尤度場作成に必要な確率を求める
  double normalizePdf(double max_pdf,
                      double pdf); // 足して1になるように確率を修正する

  double likelihood_dist_;    // 尤度場の距離
  double likelihood_dist_sq_; // 尤度場の距離
  uint32_t width_;            // 受け取ったマップのwidth
  uint32_t height_;           // 受け取ったマップのheight
  double resolution_;         // 受け取ったマップの解像度
  double origin_x_;           // 受け取ったマップの原点x
  double origin_y_;           // 受け取ったマップの原点y
  std::vector<double> data_;  // 受け取ったマップの各画素の情報
  bool create_;
  map::SparseOccupancyGridMap smap_;
  PointCloud scan_hit_cells_;
  PointCloud scan_pass_cells_;
};
} // namespace mcl

#endif