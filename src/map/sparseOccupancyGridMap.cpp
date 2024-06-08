// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include "ike_slam/map/sparseOccupancyGridMap.hpp"

#include <iostream>
#include <vector>

namespace mcl {
SparseOccupancyGridMap::SparseOccupancyGridMap()
    : max_width_(0), max_height_(0), min_width_(0), min_height_(0) {}
SparseOccupancyGridMap::~SparseOccupancyGridMap() {}

void SparseOccupancyGridMap::addCell(int x, int y, double value) {
  Index idx{x, y};
  map_data_[idx] = Cell(x, y, value);
  updateMapSize(x, y);
}

void SparseOccupancyGridMap::expandMap(int new_width, int new_height) {
  max_width_ = std::max(max_width_, new_width);
  max_height_ = std::max(max_height_, new_height);
}

void SparseOccupancyGridMap::updateMapSize(int x, int y) {
  min_width_ = std::min(min_width_, x);
  min_height_ = std::min(min_height_, y);
  max_width_ = std::max(max_width_, x);
  max_height_ = std::max(max_height_, y);
}

void SparseOccupancyGridMap::fromOccupancyGrid(
    const int width, const int height, const std::vector<double> &map_data) {
  map_data_.clear();
  max_width_ = width;
  max_height_ = height;
  for (int y = 0; y < max_height_; ++y) {
    for (int x = 0; x < max_width_; ++x) {
      int index = y * max_width_ + x;
      double value = map_data[index];
      if (value != -1) {
        addCell(x, y, value);
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid SparseOccupancyGridMap::toOccupancyGrid() const {
  nav_msgs::msg::OccupancyGrid grid;
  int grid_width = max_width_ - min_width_ + 1;
  int grid_height = max_height_ - min_height_ + 1;

  grid.info.width = grid_width;
  grid.info.height = grid_height;
  grid.data.resize(grid_width * grid_height, -1);

  grid.info.origin.position.x = min_width_;
  grid.info.origin.position.y = min_height_;

  for (auto &cell : map_data_) {
    int adjusted_x = cell.first.x - min_width_;
    int adjusted_y = cell.first.y - min_height_;
    int index = adjusted_y * grid_width + adjusted_x;
    if (index >= 0 && static_cast<size_t>(index) < grid.data.size()) {
      grid.data[index] = cell.second.value;
    }
  }

  return grid;
}

} // namespace mcl