// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/particle_filter.hpp"
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/algorithms/dist.h>
#include <limits>
#include <stdexcept>

namespace emcl3d {
struct Map::Impl {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
  using Tree = flann::KDTreeSingleIndex<flann::L2_Simple<float>>;
  std::vector<float> coordinates;
  std::unique_ptr<Tree> tree;
  Eigen::Vector3d minimum, maximum;
  bool loaded = false;
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
  filter.filter(*next->cloud);
  pcl::PointXYZ low, high;
  pcl::getMinMax3D(*next->cloud, low, high);
  next->minimum = low.getVector3fMap().cast<double>();
  next->maximum = high.getVector3fMap().cast<double>();
  next->coordinates.reserve(next->cloud->size() * 3);
  for (const auto &p : *next->cloud)
    for (float value : {p.x, p.y, p.z}) next->coordinates.push_back(value);
  // Keep PCL's float XYZ metric and 15-point exact KD-tree leaves, but avoid
  // its per-query conversion, temporary vectors, and result-index mapping.
  next->tree = std::make_unique<Impl::Tree>(
      flann::Matrix<float>(next->coordinates.data(), next->cloud->size(), 3),
      flann::KDTreeSingleIndexParams(15));
  next->tree->buildIndex();
  next->loaded = true;
  impl_ = std::move(next);
}
double Map::distance(const Eigen::Vector3d &p) const {
  if (!impl_->loaded) throw std::logic_error("Map is not loaded");
  const float query[] = {float(p.x()), float(p.y()), float(p.z())};
  thread_local flann::KNNSimpleResultSet<float> result(1);
  result.clear();
  impl_->tree->findNeighbors(result, query, flann::SearchParams(-1, 0));
  if (!result.full())
    return std::numeric_limits<double>::infinity();
  return std::sqrt(result.worstDist());
}
bool Map::contains(const Eigen::Vector3d &p, double margin) const {
  return impl_->loaded && (p.array() >= impl_->minimum.array() - margin).all() &&
         (p.array() <= impl_->maximum.array() + margin).all();
}
}  // namespace emcl3d
