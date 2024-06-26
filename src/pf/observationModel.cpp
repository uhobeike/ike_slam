// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_slam/pf/observationModel.hpp"
#include "ike_slam/map/sparseOccupancyGridMap.hpp"
#include <execution>

namespace mcl {
ObservationModel::ObservationModel(bool publish_particles_scan_match_point)
    : marginal_likelihood_(0.),
      publish_particles_scan_match_point_(publish_particles_scan_match_point) {
  std::cerr << "Run ObservationModel::ObservationModel."
            << "\n";

  std::cerr << "Done ObservationModel::ObservationModel."
            << "\n";
};
ObservationModel::~ObservationModel(){};

void ObservationModel::update(std::vector<Particle> &particles,
                              const Scan &scan) {
  // std::cerr << "Run ObservationModel::update."
  //           << "\n";

  particles_scan_match_point_.clear();
  double sum_score = 0.;
  for (auto &p : particles) {
    auto particle_weight = calculateParticleWeight(p, scan);
    p.weight *= std::abs(particle_weight);
    sum_score += particle_weight;
  }

  marginal_likelihood_ = sum_score / (particles.size() * scan.ranges.size());

  // std::cerr << "Done ObservationModel::update."
  //           << "\n";
}

double ObservationModel::calculateParticleWeight(const Particle p,
                                                 const Scan &scan) {
  // std::cerr << "Run ObservationModel::calculateParticleWeight."
  //           << "\n";

  double particle_weight = 0.;
  double scan_angle_increment = scan.angle_min;

  std::vector<std::pair<double, double>> hits_xy;
  hits_xy.reserve(scan.ranges.size());

  for (auto scan_range : scan.ranges) {
    scan_angle_increment += scan.angle_increment;
    if (std::isinf(scan_range) || std::isnan(scan_range))
      continue;

    hits_xy.push_back(std::make_pair(
        p.pose.position.x +
            scan_range * cos(p.pose.euler.yaw + scan_angle_increment),
        p.pose.position.y +
            scan_range * sin(p.pose.euler.yaw + scan_angle_increment)));
  }

  if (publish_particles_scan_match_point_) {
    particles_scan_match_point_ = hits_xy;
  }

  auto probs = getProbsFromLikelihoodMap(p, hits_xy);
  particle_weight += std::reduce(probs.begin(), probs.end());

  // std::cerr << "Done ObservationModel::calculateParticleWeight."
  //           << "\n";
  return particle_weight;
}

double ObservationModel::getProbFromLikelihoodMap(const Particle &p, double x,
                                                  double y) {
  // std::cerr << "Run ObservationModel::getProbFromLikelihoodMap."
  //           << "\n";

  int cell_x = x / p.map->resolution_;
  int cell_y = y / p.map->resolution_;

  // std::cerr << "Done ObservationModel::getProbFromLikelihoodMap."
  //           << "\n";

  return p.map->smap_.getValueFromCell<double>(cell_x, cell_y, true);
}

std::vector<double> ObservationModel::getProbsFromLikelihoodMap(
    const Particle &p, std::vector<std::pair<double, double>> &points) {
  // std::cerr << "Run ObservationModel::getProbsFromLikelihoodMap."
  //           << "\n";

  auto cells = std::move(points);

  for (auto &cell : cells) {
    cell.first /= p.map->resolution_;
    cell.second /= p.map->resolution_;
  }

  // std::cerr << "Done ObservationModel::getProbsFromLikelihoodMap."
  //           << "\n";

  return p.map->smap_.getValuesFromCells<double>(cells, true);
}

} // namespace mcl