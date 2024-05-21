#ifndef IKE_SLAM__SPARSEOCCUPANCYGRIDMAP_HPP_
#define IKE_SLAM__SPARSEOCCUPANCYGRIDMAP_HPP_

#include <iterator>
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
    return std::hash<int>{}(idx.x) ^ (std::hash<int>{}(idx.y) << 1);
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
    typename std::unordered_map<Index, Cell, Index>::iterator it_;
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
  void expandMap(int new_width, int new_height);
  void fromOccupancyGrid(const int width, const int height,
                         const std::vector<double> map_data);
  nav_msgs::msg::OccupancyGrid toOccupancyGrid() const;

private:
  void updateMapSize(int x, int y);
  std::unordered_map<Index, Cell, IndexHash> map_data_;
  int width_;
  int height_;
};

template <typename T>
T SparseOccupancyGridMap::getValueFromCell(int x, int y,
                                           bool use_likelihoodField) const {
  Index idx{x, y};
  auto it = map_data_.find(idx);
  if (it != map_data_.end()) {
    return static_cast<T>(it->second.value);
  }

  if (use_likelihoodField)
    return static_cast<T>(1.0e-10);
  else
    return static_cast<T>(-1);
}

} // namespace mcl

#endif // IKE_SLAM__SPARSEOCCUPANCYGRIDMAP_HPP_
