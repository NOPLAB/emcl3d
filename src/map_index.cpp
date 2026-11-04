// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "map_index.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace emcl3d::detail {
void MapIndex::build(pcl::PointCloud<pcl::PointXYZ> &cloud) {
  nodes.reserve(4 * cloud.size() / leaf_size + 1);
  blocks.reserve((cloud.size() + 3) / 4 + 2 * cloud.size() / leaf_size + 1);
  buildRange(cloud, 0, cloud.size());
}
size_t MapIndex::buildRange(pcl::PointCloud<pcl::PointXYZ> &cloud, size_t first, size_t last) {
  const size_t id = nodes.size();
  nodes.emplace_back();
  Eigen::Array4f low = Eigen::Array4f::Constant(std::numeric_limits<float>::infinity());
  Eigen::Array4f high = -low;
  for (size_t i = first; i < last; ++i) {
    const auto &p = cloud[i];
    const Eigen::Array4f point(p.x, p.y, p.z, 0);
    low = low.min(point);
    high = high.max(point);
  }
  nodes[id].low = low;
  nodes[id].high = high;
  if (last - first <= leaf_size) {
    nodes[id].first = blocks.size();
    nodes[id].count = (last - first + 3) / 4;
    for (size_t i = first; i < last; i += 4) {
      Block block = Block::Constant(std::numeric_limits<float>::infinity());
      for (size_t lane = 0; lane < 4 && i + lane < last; ++lane)
        block.col(lane) = cloud[i + lane].getVector3fMap().array();
      blocks.push_back(block);
    }
  } else {
    Eigen::Index axis;
    (high - low).head<3>().maxCoeff(&axis);
    const size_t middle = first + (last - first) / 2;
    std::nth_element(cloud.begin() + first, cloud.begin() + middle, cloud.begin() + last,
        [axis](const auto &a, const auto &b) { return a.data[axis] < b.data[axis]; });
    const size_t left = buildRange(cloud, first, middle);
    const size_t right = buildRange(cloud, middle, last);
    nodes[id].left = left;
    nodes[id].right = right;
  }
  return id;
}
}  // namespace emcl3d::detail
