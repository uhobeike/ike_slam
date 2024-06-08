// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__PARTICLE_HPP_
#define IKE_SLAM__PARTICLE_HPP_

#include "ike_slam/pf/likelihoodField.hpp"
#include "ike_slam/pf/pose.hpp"

struct Particle {
  Pose pose;

  double weight;
  std::shared_ptr<mcl::LikelihoodField> map;
};

#endif // IKE_SLAM__PARTICLE_HPP_