// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/particle_filter.hpp"
#include "map_index.hpp"
#ifdef EMCL3D_ENABLE_VULKAN
#include "vulkan_distance.hpp"
#endif
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <omp.h>

namespace emcl3d {
struct Map::Impl {
  detail::MapIndex index;
  Eigen::Vector3d minimum, maximum;
  bool loaded = false;
  bool use_gpu = false;
#ifdef EMCL3D_ENABLE_VULKAN
  std::unique_ptr<detail::VulkanDistance> gpu;
#endif
};
Map::Map(const std::string &compute_backend) : impl_(std::make_unique<Impl>()) {
  if (compute_backend != "cpu" && compute_backend != "vulkan")
    throw std::invalid_argument("compute_backend must be cpu or vulkan");
  impl_->use_gpu = compute_backend == "vulkan";
#ifndef EMCL3D_ENABLE_VULKAN
  if (impl_->use_gpu)
    throw std::runtime_error("Vulkan backend was not built; set EMCL3D_ENABLE_VULKAN=ON");
#endif
}
Map::~Map() = default;
void Map::load(const std::string &path, double voxel_size) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0)
    throw std::invalid_argument("map_voxel_size must be positive");
  auto next = std::make_unique<Impl>();
  next->use_gpu = impl_->use_gpu;
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
  next->index.build(cloud);
#ifdef EMCL3D_ENABLE_VULKAN
  if (next->use_gpu) next->gpu = std::make_unique<detail::VulkanDistance>(next->index);
#endif
  next->loaded = true;
  impl_ = std::move(next);
}
double Map::distance(const Eigen::Vector3d &p, float squared_limit) const {
  if (!impl_->loaded) throw std::logic_error("Map is not loaded");
  const Eigen::Array4f query(float(p.x()), float(p.y()), float(p.z()), 0);
  return std::sqrt(impl_->index.squaredDistance(query, squared_limit));
}
void Map::squaredDistances(const std::vector<Eigen::Vector4f> &queries, float squared_limit,
    std::vector<float> &output, int num_threads) const {
  if (!impl_->loaded) throw std::logic_error("Map is not loaded");
#ifdef EMCL3D_ENABLE_VULKAN
  if (impl_->use_gpu) {
    impl_->gpu->squaredDistances(queries, squared_limit, output);
    return;
  }
#endif
  output.resize(queries.size());
#pragma omp parallel for num_threads(num_threads > 0 ? num_threads : omp_get_max_threads())
  for (size_t i = 0; i < queries.size(); ++i) {
    const Eigen::Array4f query(queries[i].x(), queries[i].y(), queries[i].z(), 0);
    output[i] = impl_->index.squaredDistance(query, squared_limit);
  }
}
bool Map::usesGpu() const { return impl_->use_gpu; }
std::pair<Eigen::Vector3d, Eigen::Vector3d> Map::bounds() const {
  if (!impl_->loaded) throw std::logic_error("Map is not loaded");
  return {impl_->minimum, impl_->maximum};
}
std::string Map::deviceName() const {
  if (!impl_->loaded) throw std::logic_error("Map is not loaded");
#ifdef EMCL3D_ENABLE_VULKAN
  if (impl_->use_gpu) return impl_->gpu->deviceName();
#endif
  return "CPU";
}
bool Map::contains(const Eigen::Vector3d &p, double margin) const {
  return impl_->loaded && (p.array() >= impl_->minimum.array() - margin).all() &&
         (p.array() <= impl_->maximum.array() + margin).all();
}
}  // namespace emcl3d
