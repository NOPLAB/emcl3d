// SPDX-License-Identifier: LGPL-3.0-only
#include "vulkan_context.hpp"
#include <cstring>
#include <limits>
#include <stdexcept>

namespace emcl3d::detail {
VulkanContext::VulkanContext() {
  vk::ApplicationInfo app("emcl3d", 1, "emcl3d", 1, VK_API_VERSION_1_1);
  instance = vk::createInstanceUnique(vk::InstanceCreateInfo({}, &app));
  for (auto candidate : instance->enumeratePhysicalDevices()) {
    const auto p = candidate.getProperties();
    if (p.apiVersion < VK_API_VERSION_1_1 ||
        (p.deviceType != vk::PhysicalDeviceType::eIntegratedGpu &&
         p.deviceType != vk::PhysicalDeviceType::eDiscreteGpu)) continue;
    const auto families = candidate.getQueueFamilyProperties();
    for (uint32_t i = 0; i < families.size(); ++i) {
      if (families[i].queueCount && (families[i].queueFlags & vk::QueueFlagBits::eCompute)) {
        physical = candidate;
        properties = p;
        family = i;
        break;
      }
    }
    if (physical) break;
  }
  if (!physical) throw std::runtime_error("Vulkan: no Vulkan 1.1 integrated/discrete compute GPU");
  const auto &limits = properties.limits;
  if (limits.maxComputeWorkGroupInvocations < 64 || limits.maxComputeWorkGroupSize[0] < 64 ||
      limits.maxPerStageDescriptorStorageBuffers < 4 || limits.maxDescriptorSetStorageBuffers < 4 ||
      limits.maxPushConstantsSize < 8)
    throw std::runtime_error("Vulkan: device compute limits do not support distance shader");
  const float priority = 1;
  vk::DeviceQueueCreateInfo queue_info({}, family, 1, &priority);
  device = physical.createDeviceUnique(vk::DeviceCreateInfo({}, 1, &queue_info));
  queue = device->getQueue(family, 0);
  memory_properties_ = physical.getMemoryProperties();
  command_pool = device->createCommandPoolUnique(
      vk::CommandPoolCreateInfo(vk::CommandPoolCreateFlagBits::eResetCommandBuffer, family));
}

std::unique_ptr<VulkanBuffer> VulkanContext::buffer(vk::DeviceSize bytes,
    BufferLocation location) const {
  if (!bytes || bytes > properties.limits.maxStorageBufferRange)
    throw std::length_error("Vulkan: storage buffer exceeds device limit");
  auto result = std::make_unique<VulkanBuffer>();
  result->device = *device;
  result->capacity = bytes;
  result->buffer = device->createBufferUnique(vk::BufferCreateInfo(
      {}, bytes, vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferSrc |
          vk::BufferUsageFlagBits::eTransferDst, vk::SharingMode::eExclusive));
  const auto requirements = device->getBufferMemoryRequirements(*result->buffer);
  uint32_t type = UINT32_MAX;
  int best_rank = -1;
  for (uint32_t i = 0; i < memory_properties_.memoryTypeCount; ++i) {
    const auto flags = memory_properties_.memoryTypes[i].propertyFlags;
    if (!(requirements.memoryTypeBits & (1u << i))) continue;
    int rank = 0;
    switch (location) {
      case BufferLocation::Host:
        if (!(flags & vk::MemoryPropertyFlagBits::eHostVisible)) continue;
        rank = 2 * bool(flags & vk::MemoryPropertyFlagBits::eHostCoherent) +
            bool(flags & vk::MemoryPropertyFlagBits::eHostCached);
        break;
      case BufferLocation::Device:
        if (!(flags & vk::MemoryPropertyFlagBits::eDeviceLocal)) continue;
        // Prefer ordinary VRAM over a limited host-visible BAR heap.
        rank = !(flags & vk::MemoryPropertyFlagBits::eHostVisible);
        break;
    }
    if (rank > best_rank) { type = i; best_rank = rank; }
  }
  if (type == UINT32_MAX) throw std::runtime_error("Vulkan: no suitable storage memory");
  result->coherent = bool(memory_properties_.memoryTypes[type].propertyFlags &
                          vk::MemoryPropertyFlagBits::eHostCoherent);
  result->memory = device->allocateMemoryUnique(vk::MemoryAllocateInfo(requirements.size, type));
  device->bindBufferMemory(*result->buffer, *result->memory, 0);
  if (memory_properties_.memoryTypes[type].propertyFlags & vk::MemoryPropertyFlagBits::eHostVisible)
    result->mapped = device->mapMemory(*result->memory, 0, VK_WHOLE_SIZE);
  return result;
}

void VulkanContext::write(const VulkanBuffer &target, const void *data, size_t bytes) const {
  if (target.mapped) {
    std::memcpy(target.mapped, data, bytes);
    if (!target.coherent)
      device->flushMappedMemoryRanges(vk::MappedMemoryRange(*target.memory, 0, VK_WHOLE_SIZE));
    return;
  }
  auto staging = buffer(bytes);
  write(*staging, data, bytes);
  auto commands = device->allocateCommandBuffersUnique(vk::CommandBufferAllocateInfo(
      *command_pool, vk::CommandBufferLevel::ePrimary, 1));
  const auto command = *commands.front();
  command.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
  command.copyBuffer(*staging->buffer, *target.buffer, vk::BufferCopy(0, 0, bytes));
  const vk::BufferMemoryBarrier visible(
      vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead,
      VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, *target.buffer, 0, VK_WHOLE_SIZE);
  command.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
      vk::PipelineStageFlagBits::eComputeShader, {}, {}, visible, {});
  command.end();
  submit(command);
}

void VulkanContext::read(const VulkanBuffer &buffer, void *data, size_t bytes) const {
  if (!buffer.coherent)
    device->invalidateMappedMemoryRanges(vk::MappedMemoryRange(*buffer.memory, 0, VK_WHOLE_SIZE));
  std::memcpy(data, buffer.mapped, bytes);
}

void VulkanContext::submit(vk::CommandBuffer command) const {
  auto fence = device->createFenceUnique({});
  queue.submit(vk::SubmitInfo(0, nullptr, nullptr, 1, &command), *fence);
  const auto result = device->waitForFences(*fence, true, std::numeric_limits<uint64_t>::max());
  if (result != vk::Result::eSuccess)
    throw std::runtime_error("Vulkan: distance dispatch fence failed");
}
}  // namespace emcl3d::detail
