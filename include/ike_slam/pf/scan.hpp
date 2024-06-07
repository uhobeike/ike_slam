// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__SCAN_HPP_
#define IKE_SLAM__SCAN_HPP_

#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

struct Scan {
  float angle_min;
  float angle_max;
  float angle_increment;
  float range_min;
  float range_max;
  std::vector<float> ranges;

  Scan &operator=(const sensor_msgs::msg::LaserScan &scan) {
    angle_min = scan.angle_min;
    angle_max = scan.angle_max;
    angle_increment = scan.angle_increment;
    range_min = scan.range_min;
    range_max = scan.range_max;
    ranges = std::move(scan.ranges);
    return *this;
  }
};

#endif // IKE_SLAM__SCAN_HPP_
