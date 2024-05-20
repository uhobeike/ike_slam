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

int SparseOccupancyGridMap::getValueFromCell(int x, int y) const {
  Index idx{x, y};
  auto it = map_data_.find(idx);
  if (it != map_data_.end()) {
    return it->second.value;
  }
  return -1;
}

void SparseOccupancyGridMap::expandMap(int new_width, int new_height) {
  if (new_width > width_) {
    width_ = new_width;
  }
  if (new_height > height_) {
    height_ = new_height;
  }
}

void SparseOccupancyGridMap::fromOccupancyGrid(
    const int width, const int height, const std::vector<double> map_data) {
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
  if (x >= width_) {
    width_ = x + 1;
  }
  if (y >= height_) {
    height_ = y + 1;
  }
}
} // namespace mcl