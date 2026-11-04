// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

namespace pcl {
struct PointXYZ;
template<class Point> class PointCloud;
}

namespace emcl3d::detail {
// Shared geometry; GPU upload converts node metadata, not point coordinates.
class MapIndex {
public:
  static constexpr size_t leaf_size = 32;
  using Block = Eigen::Array<float, 3, 4, Eigen::RowMajor>;
  struct alignas(64) Node {
    Eigen::Array4f low, high;
    size_t first = 0, count = 0, left = 0, right = 0;
  };
  std::vector<Node> nodes;
  std::vector<Block> blocks;

  void build(pcl::PointCloud<pcl::PointXYZ> &cloud);
  float squaredDistance(const Eigen::Array4f &query, float limit) const;

private:
  size_t buildRange(pcl::PointCloud<pcl::PointXYZ> &cloud, size_t first, size_t last);
  static float lowerBound(const Node &node, const Eigen::Array4f &query);
  void nearest(size_t id, const Eigen::Array4f &query, float &best) const;
};
}  // namespace emcl3d::detail

#include "map_query.hpp"
