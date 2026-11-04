// SPDX-License-Identifier: LGPL-3.0-only
#pragma once
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace emcl3d::detail {
class MapIndex;
class VulkanDistance {
public:
  explicit VulkanDistance(const MapIndex &index);
  ~VulkanDistance();
  void squaredDistances(const std::vector<Eigen::Vector4f> &queries, float limit,
                        std::vector<float> &output);
  std::string deviceName() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace emcl3d::detail
