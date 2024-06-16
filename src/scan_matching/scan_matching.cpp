// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_slam/scan_matching/scan_matching.hpp"

namespace scan_matching {
ScanMatching::ScanMatching() {}
ScanMatching::~ScanMatching() {}

double ScanMatching::getEuclideanDistance(const Point &point1,
                                          const Point &point2) {
  return (point1 - point2).norm();
}

std::pair<Eigen::Matrix2d, Eigen::Vector2d>
ScanMatching::icp(PointCloud source, const PointCloud &target,
                  int max_iterations, double tolerance) {
  Eigen::Matrix2d R_total = Eigen::Matrix2d::Identity();
  Eigen::Vector2d t_total(0, 0);

  for (int iter = 0; iter < max_iterations; ++iter) {
    std::vector<int> correspondences = findClosestPoints(source, target);
    PointCloud target_matched(source.size());
    for (size_t i = 0; i < source.size(); ++i) {
      target_matched[i] = target[correspondences[i]];
    }

    Point source_centroid = computeCentroid(source);
    Point target_centroid = computeCentroid(target_matched);

    PointCloud source_centered(source.size()),
        target_centered(target_matched.size());
    for (size_t i = 0; i < source.size(); ++i) {
      source_centered[i] = source[i] - source_centroid;
      target_centered[i] = target_matched[i] - target_centroid;
    }

    Eigen::Matrix2d R = computeRotationMatrix(source_centered, target_centered);
    Eigen::Vector2d t = target_centroid - R * source_centroid;

    source = transformPointCloud(source, R, t);

    R_total = R * R_total;
    t_total = R * t_total + t;

    if ((source_centroid - target_centroid).norm() < tolerance) {
      break;
    }
  }

  return std::make_pair(R_total, t_total);
}

std::vector<int> ScanMatching::findClosestPoints(const PointCloud &source,
                                                 const PointCloud &target) {
  std::vector<int> correspondences(source.size());
  for (size_t i = 0; i < source.size(); ++i) {
    double min_dist = std::numeric_limits<double>::max();
    int min_index = -1;
    for (size_t j = 0; j < target.size(); ++j) {
      double dist = getEuclideanDistance(source[i], target[j]);
      if (dist < min_dist) {
        min_dist = dist;
        min_index = j;
      }
    }
    correspondences[i] = min_index;
  }
  return correspondences;
}

Point ScanMatching::computeCentroid(const PointCloud &points) {
  Point centroid(0, 0);
  for (const auto &point : points) {
    centroid += point;
  }
  centroid /= points.size();
  return centroid;
}

Eigen::Matrix2d ScanMatching::computeRotationMatrix(const PointCloud &source,
                                                    const PointCloud &target) {
  double A = 0, B = 0;
  for (size_t i = 0; i < source.size(); ++i) {
    A += source[i].x() * target[i].y() - source[i].y() * target[i].x();
    B += source[i].x() * target[i].x() + source[i].y() * target[i].y();
  }
  double theta = atan2(A, B);
  Eigen::Matrix2d R;
  R << cos(theta), -sin(theta), sin(theta), cos(theta);
  return R;
}

PointCloud ScanMatching::transformPointCloud(const PointCloud &points,
                                             const Eigen::Matrix2d &R,
                                             const Eigen::Vector2d &t) {
  PointCloud transformed(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    transformed[i] = R * points[i] + t;
  }
  return transformed;
}

} // namespace scan_matching