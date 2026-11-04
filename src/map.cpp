// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/particle_filter.hpp"
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace emcl3d {
struct Map::Impl {
  static constexpr size_t leaf_size = 32;
  using Block = Eigen::Array<float, 3, 4, Eigen::RowMajor>;
  struct alignas(64) Node {
    Eigen::Array4f low, high;
    size_t first = 0, count = 0, left = 0, right = 0;
  };
  std::vector<Node> nodes;
  std::vector<Block> blocks;
  Eigen::Vector3d minimum, maximum;
  bool loaded = false;

  size_t build(pcl::PointCloud<pcl::PointXYZ> &cloud, size_t first, size_t last) {
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
      const size_t left = build(cloud, first, middle);
      const size_t right = build(cloud, middle, last);
      nodes[id].left = left;
      nodes[id].right = right;
    }
    return id;
  }

  static float lower_bound(const Node &node, const Eigen::Array4f &query) {
    const Eigen::Array4f gap = (node.low - query).max(0) + (query - node.high).max(0);
    return gap.square().sum();
  }
  void nearest(size_t id, const Eigen::Array4f &query, float &best) const {
    const auto &node = nodes[id];
    if (node.count) {
      for (size_t i = node.first; i < node.first + node.count; ++i) {
        const auto &b = blocks[i];
        const Eigen::Array<float, 1, 4> squared =
            (b.row(0) - query[0]).square() + (b.row(1) - query[1]).square() +
            (b.row(2) - query[2]).square();
        best = std::min(best, squared.minCoeff());
      }
      return;
    }
    size_t first = node.left, second = node.right;
    float a = lower_bound(nodes[first], query), b = lower_bound(nodes[second], query);
    if (b < a) { std::swap(a, b); std::swap(first, second); }
    // Conservative slack prevents float box roundoff from pruning a tied neighbor.
    constexpr float slack = 1 + 16 * std::numeric_limits<float>::epsilon();
    if (!std::isfinite(a) || a <= best * slack) nearest(first, query, best);
    if (!std::isfinite(b) || b <= best * slack) nearest(second, query, best);
  }
};
Map::Map() : impl_(std::make_unique<Impl>()) {}
Map::~Map() = default;
void Map::load(const std::string &path, double voxel_size) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0)
    throw std::invalid_argument("map_voxel_size must be positive");
  auto next = std::make_unique<Impl>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(path, *input) < 0)
    throw std::runtime_error("Cannot load PCD map: " + path);
  pcl::PointCloud<pcl::PointXYZ>::Ptr finite(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &p : *input)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      finite->push_back(p);
  if (finite->empty()) throw std::runtime_error("PCD map contains no finite points");
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(finite);
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  filter.filter(cloud);
  if (cloud.empty()) throw std::runtime_error("Voxelized map contains no points");
  pcl::PointXYZ low, high;
  pcl::getMinMax3D(cloud, low, high);
  next->minimum = low.getVector3fMap().cast<double>();
  next->maximum = high.getVector3fMap().cast<double>();
  next->nodes.reserve(4 * cloud.size() / Impl::leaf_size + 1);
  next->blocks.reserve((cloud.size() + 3) / 4 + 2 * cloud.size() / Impl::leaf_size + 1);
  next->build(cloud, 0, cloud.size());
  next->loaded = true;
  impl_ = std::move(next);
}
double Map::distance(const Eigen::Vector3d &p, float squared_limit) const {
  if (!impl_->loaded) throw std::logic_error("Map is not loaded");
  const Eigen::Array4f query(float(p.x()), float(p.y()), float(p.z()), 0);
  float squared = squared_limit;
  impl_->nearest(0, query, squared);
  if (!(squared < squared_limit))
    return std::numeric_limits<double>::infinity();
  return std::sqrt(squared);
}
bool Map::contains(const Eigen::Vector3d &p, double margin) const {
  return impl_->loaded && (p.array() >= impl_->minimum.array() - margin).all() &&
         (p.array() <= impl_->maximum.array() + margin).all();
}
}  // namespace emcl3d
