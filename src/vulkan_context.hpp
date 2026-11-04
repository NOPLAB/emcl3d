// SPDX-License-Identifier: LGPL-3.0-only
#pragma once
#include <vulkan/vulkan.hpp>
#include <cstddef>
#include <memory>
#include <string>

namespace emcl3d::detail {
enum class BufferLocation { Host, Device };
// Destruction order is intentional: buffers precede memory, device precedes instance.
struct VulkanBuffer {
  vk::Device device;
  vk::UniqueDeviceMemory memory;
  vk::UniqueBuffer buffer;
  vk::DeviceSize capacity = 0;
  bool coherent = false;
  void *mapped = nullptr;
  ~VulkanBuffer() { if (mapped) device.unmapMemory(*memory); }
};

class VulkanContext {
public:
  VulkanContext();
  std::unique_ptr<VulkanBuffer> buffer(vk::DeviceSize bytes,
      BufferLocation location = BufferLocation::Host) const;
  void write(const VulkanBuffer &buffer, const void *data, size_t bytes) const;
  void read(const VulkanBuffer &buffer, void *data, size_t bytes) const;
  void submit(vk::CommandBuffer command) const;
  vk::UniqueInstance instance;
  vk::PhysicalDevice physical;
  vk::PhysicalDeviceProperties properties;
  vk::UniqueDevice device;
  vk::Queue queue;
  uint32_t family = 0;
  vk::UniqueCommandPool command_pool;
private:
  vk::PhysicalDeviceMemoryProperties memory_properties_;
};
}  // namespace emcl3d::detail
