// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "mcl3d/particle_filter.hpp"
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <limits>
#include <stdexcept>

namespace mcl3d {
struct Map::Impl {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
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
  next->tree.setInputCloud(next->cloud);
  next->loaded = true;
  impl_ = std::move(next);
}
double Map::distance(const Eigen::Vector3d &p) const {
  if (!impl_->loaded) throw std::logic_error("Map is not loaded");
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  if (impl_->tree.nearestKSearch(pcl::PointXYZ(p.x(), p.y(), p.z()), 1, indices, distances) != 1)
    return std::numeric_limits<double>::infinity();
  return std::sqrt(distances[0]);
}
bool Map::contains(const Eigen::Vector3d &p, double margin) const {
  return impl_->loaded && (p.array() >= impl_->minimum.array() - margin).all() &&
         (p.array() <= impl_->maximum.array() + margin).all();
}
}  // namespace mcl3d
