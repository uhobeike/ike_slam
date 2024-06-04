// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef IKE_SLAM__POSE_HPP_
#define IKE_SLAM__POSE_HPP_

struct Euler {
  double yaw;
  Euler(double yaw = 0.0) : yaw(yaw) {}
};

struct Point {
  double x;
  double y;
  Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}
};

struct Pose {
  Point position;
  Euler euler;
  Pose(double x = 0.0, double y = 0.0, double yaw = 0.0)
      : position(x, y), euler(yaw) {}
};

#endif // IKE_SLAM__POSE_HPP_