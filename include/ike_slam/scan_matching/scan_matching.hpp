// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__SCAN_MATCHING_HPP_
#define IKE_SLAM__SCAN_MATCHING_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <iterator>
#include <limits>
#include <vector>

namespace scan_matching {

typedef Eigen::Vector2d Point;
typedef std::vector<Eigen::Vector2d> PointCloud;

class ScanMatching {
public:
  ScanMatching();
  ~ScanMatching();

  std::pair<Eigen::Matrix2d, Eigen::Vector2d> icp(PointCloud source,
                                                  const PointCloud &target,
                                                  int max_iterations = 100,
                                                  double tolerance = 1e-6);

private:
  double getEuclideanDistance(const Point &point1, const Point &point2);

  std::vector<int> findClosestPoints(const PointCloud &source,
                                     const PointCloud &target);

  Point computeCentroid(const PointCloud &points);

  Eigen::Matrix2d computeRotationMatrix(const PointCloud &source,
                                        const PointCloud &target);

  PointCloud transformPointCloud(const PointCloud &points,
                                 const Eigen::Matrix2d &R,
                                 const Eigen::Vector2d &t);
};

} // namespace scan_matching

#endif // IKE_SLAM__SCAN_MATCHING_HPP_
