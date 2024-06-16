// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_slam/mapping/mapping.hpp"

#include <cmath>
#include <limits>

namespace mapping {
Mapping::Mapping(double resolution) : resolution_(resolution){};
Mapping::~Mapping(){};

void Mapping::gridMapping(std::vector<Particle> &particles, const Scan &scan) {
  for (auto &particle : particles) {
    auto [scan_hit_cells, scan_pass_cells] =
        getUpdateCells(particle.pose, scan);
    upadateCells(particle.map, scan_hit_cells, scan_pass_cells);
  }
}

std::tuple<std::vector<std::pair<double, double>>,
           std::vector<std::pair<double, double>>>
Mapping::getUpdateCells(const Pose pose, const Scan &scan) {
  auto &&scan_hit_cells = calcScanHitCells(pose, scan);
  return {std::move(scan_hit_cells), calcScanPassCells(pose, scan_hit_cells)};
}

void Mapping::upadateCells(
    std::shared_ptr<mcl::LikelihoodField> likelihood_field,
    std::vector<std::pair<double, double>> &scan_hit_cells,
    std::vector<std::pair<double, double>> &scan_pass_cells) {
  for (auto &scan_pass_cell : scan_pass_cells) {
    likelihood_field->smap_.addCell(scan_pass_cell.first, scan_pass_cell.second,
                                    0.01);
  }

  for (auto &scan_hit_cell : scan_hit_cells) {
    likelihood_field->smap_.addCell(scan_hit_cell.first, scan_hit_cell.second,
                                    100.0);
    // likelihood_field->createLikelihoodField(scan_hit_cell.first,
    //                                         scan_hit_cell.second);
  }

  PointCloud pointcloud_pass;
  for (const auto &p : scan_pass_cells) {
    Eigen::Vector2d vec(p.first, p.second);
    pointcloud_pass.push_back(vec);
  }

  PointCloud pointcloud_hit;
  for (const auto &p : scan_hit_cells) {
    Eigen::Vector2d vec(p.first, p.second);
    pointcloud_hit.push_back(vec);
  }

  likelihood_field->scan_pass_cells_ = std::move(pointcloud_pass);
  likelihood_field->scan_hit_cells_ = std::move(pointcloud_hit);
}

std::vector<std::pair<double, double>>
Mapping::calcScanHitCells(const Pose pose, const Scan &scan) {
  std::vector<std::pair<double, double>> hits_xy(scan.ranges.size());

  double scan_angle_increment = scan.angle_min;

  for (auto scan_range : scan.ranges) {
    scan_angle_increment += scan.angle_increment;
    if (std::isinf(scan_range) || std::isnan(scan_range))
      continue;

    hits_xy.push_back(std::make_pair(
        (pose.position.x +
         scan_range * cos(pose.euler.yaw + scan_angle_increment)) /
            resolution_,
        (pose.position.y +
         scan_range * sin(pose.euler.yaw + scan_angle_increment)) /
            resolution_));
  }

  return hits_xy;
}

std::vector<std::pair<double, double>> Mapping::calcScanPassCells(
    const Pose pose,
    const std::vector<std::pair<double, double>> &scan_hits_cells) {
  int pose_x = pose.position.x / resolution_;
  int pose_y = pose.position.y / resolution_;

  std::vector<std::pair<double, double>> line_grid_cells_set;
  for (auto &scan_hit_cell : scan_hits_cells) {
    auto line_grid_cells =
        bresenham(pose_x, pose_y, scan_hit_cell.first, scan_hit_cell.second);

    if (line_grid_cells.size() > 0) {
      line_grid_cells_set.insert(line_grid_cells_set.end(),
                                 line_grid_cells.begin(),
                                 line_grid_cells.end());
    }
  }

  return line_grid_cells_set;
}

std::vector<std::pair<double, double>> Mapping::bresenham(int x0, int y0,
                                                          int x1, int y1) {
  int delta_x = abs(x0 - x1);
  int deltta_y = abs(y0 - y1);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = delta_x - deltta_y;

  std::vector<std::pair<double, double>> line_grid_cells;

  while (true) {
    line_grid_cells.push_back(std::make_pair(x0, y0));

    if (x0 == x1 && y0 == y1)
      break;

    int e2 = err * 2;
    if (e2 > -deltta_y) {
      err -= deltta_y;
      x0 += sx;
    }
    if (e2 < deltta_y) {
      err += delta_x;
      y0 += sy;
    }
  }

  return line_grid_cells;
}

} // namespace mapping