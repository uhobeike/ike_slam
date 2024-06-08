#ifndef IKE_SLAM__SPARSEOCCUPANCYGRIDMAP_HPP_
#define IKE_SLAM__SPARSEOCCUPANCYGRIDMAP_HPP_

#define _ENABLE_ATOMIC_ALIGNMENT_FIX

#include <execution>
#include <iterator>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <unordered_map>
#include <vector>

namespace mcl {

struct Index {
  int x;
  int y;

  bool operator==(const Index &other) const {
    return x == other.x && y == other.y;
  }
};

struct IndexHash {
  std::size_t operator()(const Index &idx) const {
    std::size_t seed = 0;
    std::hash<int> hasher;

    seed ^= hasher(idx.x) + 0x9e3779b9 + (seed << 13) + (seed >> 7);
    seed ^= hasher(idx.y) + 0x85ebca6b + (seed << 15) + (seed >> 5);

    return seed;
  }
};

struct Cell {
  int x;
  int y;
  double value;

  Cell() : x(0), y(0), value(0.) {}
  Cell(int x, int y, double val) : x(x), y(y), value(val) {}
};

class SparseOccupancyGridMap {
public:
  SparseOccupancyGridMap();
  ~SparseOccupancyGridMap();

  class Iterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = Cell;
    using pointer = Cell *;
    using reference = Cell &;

    Iterator(typename std::unordered_map<Index, Cell, IndexHash>::iterator it)
        : it_(it) {}

    reference operator*() const { return it_->second; }
    pointer operator->() { return &it_->second; }
    Iterator &operator++() {
      ++it_;
      return *this;
    }
    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }
    friend bool operator==(const Iterator &a, const Iterator &b) {
      return a.it_ == b.it_;
    }
    friend bool operator!=(const Iterator &a, const Iterator &b) {
      return a.it_ != b.it_;
    }

  private:
    typename std::unordered_map<Index, Cell, IndexHash>::iterator it_;
  };

  Iterator begin() { return Iterator(map_data_.begin()); }
  Iterator end() { return Iterator(map_data_.end()); }
  using const_iterator =
      typename std::unordered_map<Index, Cell, IndexHash>::const_iterator;
  const_iterator begin() const { return map_data_.begin(); }
  const_iterator end() const { return map_data_.end(); }

  void addCell(int x, int y, double value);
  template <typename T>
  T getValueFromCell(int x, int y, bool use_likelihoodField = false) const;
  template <typename T>
  std::vector<T>
  getValuesFromCells(const std::vector<std::pair<double, double>> &points,
                     bool use_likelihoodField) const;
  void expandMap(int new_width, int new_height);
  void fromOccupancyGrid(const int width, const int height,
                         const std::vector<double> &map_data);
  nav_msgs::msg::OccupancyGrid toOccupancyGrid() const;
  nav_msgs::msg::OccupancyGrid toOccupancyGrid2() const;

  std::unordered_map<Index, Cell, IndexHash> map_data_;

private:
  void updateMapSize(int x, int y);
  int max_width_;
  int max_height_;
  int min_width_;
  int min_height_;
};
template <typename T>
inline T
SparseOccupancyGridMap::getValueFromCell(int x, int y,
                                         bool use_likelihoodField) const {
  static const double likelihood_default = static_cast<double>(1.0e-10);
  static const int8_t normal_default = static_cast<int8_t>(-1);

  Index idx{x, y};
  auto it = map_data_.find(idx);

  if (it != map_data_.end()) {
    return static_cast<T>(it->second.value);
  }

  return use_likelihoodField ? likelihood_default : normal_default;
}

template <typename T>
inline std::vector<T> SparseOccupancyGridMap::getValuesFromCells(
    const std::vector<std::pair<double, double>> &points,
    bool use_likelihoodField) const {
  std::vector<T> values(points.size());

  std::transform(
      points.begin(), points.end(), values.begin(),
      [this, use_likelihoodField](const std::pair<double, double> &point) {
        return this->template getValueFromCell<T>(point.first, point.second,
                                                  use_likelihoodField);
      });

  return values;
}

} // namespace mcl

#endif // IKE_SLAM__SPARSEOCCUPANCYGRIDMAP_HPP_
