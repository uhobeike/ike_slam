// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_slam/map/sparseOccupancyGridMap.hpp"

#include <vector>

namespace mcl {
SparseOccupancyGridMap::SparseOccupancyGridMap() : width_(0), height_(0) {}
SparseOccupancyGridMap::~SparseOccupancyGridMap() {}

void SparseOccupancyGridMap::addCell(int x, int y, double value) {
  Index idx{x, y};
  map_data_[idx] = Cell(x, y, value);
  updateMapSize(x, y);
}

void SparseOccupancyGridMap::expandMap(int new_width, int new_height) {
  width_ = std::max(width_, new_width);
  height_ = std::max(height_, new_height);
}

void SparseOccupancyGridMap::fromOccupancyGrid(
    const int width, const int height, const std::vector<double> &map_data) {
  map_data_.clear();
  width_ = width;
  height_ = height;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int index = y * width_ + x;
      double value = map_data[index];
      if (value != -1) {
        addCell(x, y, value);
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid SparseOccupancyGridMap::toOccupancyGrid() const {
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = width_;
  grid.info.height = height_;
  grid.data.resize(width_ * height_, -1);

  for (const auto &cell : map_data_) {
    int index = cell.first.y * width_ + cell.first.x;
    grid.data[index] = cell.second.value;
  }
  return grid;
}

void SparseOccupancyGridMap::updateMapSize(int x, int y) {
  width_ = std::max(width_, x + 1);
  height_ = std::max(height_, y + 1);
}
} // namespace mcl