// SPDX-License-Identifier: LGPL-3.0-only
#include "vulkan_distance.hpp"
#include "vulkan_context.hpp"
#include "map_index.hpp"
#include "map_distance_spirv.hpp"
#include <array>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>

namespace emcl3d::detail {
namespace {
struct alignas(16) DeviceNode {
  float low[4], high[4];
  uint32_t meta[4];
};
static_assert(sizeof(DeviceNode) == 48);
static_assert(sizeof(MapIndex::Block) == 48);
static_assert(sizeof(Eigen::Vector4f) == 16);

std::vector<DeviceNode> convert(const MapIndex &index) {
  if (index.nodes.empty() || index.blocks.empty() ||
      index.nodes.size() > UINT32_MAX || index.blocks.size() > UINT32_MAX / 3)
    throw std::length_error("Vulkan: empty or oversized map index");
  // Preorder children must follow their parent. This also rules out cycles.
  std::vector<uint32_t> depths(index.nodes.size(), 0);
  depths[0] = 1;
  std::vector<DeviceNode> result(index.nodes.size());
  for (size_t i = 0; i < index.nodes.size(); ++i) {
    const auto &node = index.nodes[i];
    if (!depths[i] || depths[i] > 32)
      throw std::length_error("Vulkan: map tree exceeds 32-entry traversal stack");
    if (node.count) {
      if (node.first > index.blocks.size() || node.count > index.blocks.size() - node.first)
        throw std::invalid_argument("Vulkan: invalid map leaf range");
    } else {
      for (auto child : {node.left, node.right}) {
        if (child <= i || child >= index.nodes.size() || depths[child])
          throw std::invalid_argument("Vulkan: invalid preorder map tree");
        depths[child] = depths[i] + 1;
      }
    }
    auto &dest = result[i];
    std::memcpy(dest.low, node.low.data(), sizeof(dest.low));
    std::memcpy(dest.high, node.high.data(), sizeof(dest.high));
    dest.meta[0] = static_cast<uint32_t>(node.first);
    dest.meta[1] = static_cast<uint32_t>(node.count);
    dest.meta[2] = static_cast<uint32_t>(node.left);
    dest.meta[3] = static_cast<uint32_t>(node.right);
  }
  return result;
}
}  // namespace

struct VulkanDistance::Impl {
  VulkanContext context;
  std::unique_ptr<VulkanBuffer> nodes, blocks, queries, results;
  vk::UniqueDescriptorSetLayout descriptor_layout;
  vk::UniquePipelineLayout pipeline_layout;
  vk::UniquePipeline pipeline;
  vk::UniqueDescriptorPool descriptor_pool;
  vk::DescriptorSet descriptor;
  std::vector<vk::UniqueCommandBuffer> commands;
  std::mutex mutex;

  explicit Impl(const MapIndex &index) {
    const auto converted = convert(index);
    nodes = context.buffer(converted.size() * sizeof(DeviceNode), BufferLocation::Device);
    blocks = context.buffer(index.blocks.size() * sizeof(MapIndex::Block), BufferLocation::Device);
    context.write(*nodes, converted.data(), converted.size() * sizeof(DeviceNode));
    context.write(*blocks, index.blocks.data(), index.blocks.size() * sizeof(MapIndex::Block));
    std::array<vk::DescriptorSetLayoutBinding, 4> bindings;
    for (uint32_t i = 0; i < bindings.size(); ++i)
      bindings[i] = vk::DescriptorSetLayoutBinding(
          i, vk::DescriptorType::eStorageBuffer, 1, vk::ShaderStageFlagBits::eCompute);
    descriptor_layout = context.device->createDescriptorSetLayoutUnique(
        vk::DescriptorSetLayoutCreateInfo({}, bindings));
    const vk::PushConstantRange push(vk::ShaderStageFlagBits::eCompute, 0, 8);
    pipeline_layout = context.device->createPipelineLayoutUnique(
        vk::PipelineLayoutCreateInfo({}, 1, &*descriptor_layout, 1, &push));
    auto shader = context.device->createShaderModuleUnique(
        vk::ShaderModuleCreateInfo({}, sizeof(map_distance_spirv), map_distance_spirv));
    pipeline = context.device->createComputePipelineUnique({}, vk::ComputePipelineCreateInfo(
        {}, vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eCompute,
                                             *shader, "main"), *pipeline_layout)).value;
    const vk::DescriptorPoolSize size(vk::DescriptorType::eStorageBuffer, 4);
    descriptor_pool = context.device->createDescriptorPoolUnique(
        vk::DescriptorPoolCreateInfo({}, 1, 1, &size));
    descriptor = context.device->allocateDescriptorSets(
        vk::DescriptorSetAllocateInfo(*descriptor_pool, 1, &*descriptor_layout)).front();
    commands = context.device->allocateCommandBuffersUnique(vk::CommandBufferAllocateInfo(
        *context.command_pool, vk::CommandBufferLevel::ePrimary, 1));
  }

  void run(const std::vector<Eigen::Vector4f> &input, float limit, std::vector<float> &output) {
    std::lock_guard<std::mutex> lock(mutex);
    if (input.empty()) { output.clear(); return; }
    const uint64_t groups = (uint64_t(input.size()) + 63) / 64;
    if (input.size() > UINT32_MAX ||
        groups > context.properties.limits.maxComputeWorkGroupCount[0] ||
        input.size() > context.properties.limits.maxStorageBufferRange / sizeof(Eigen::Vector4f))
      throw std::length_error("Vulkan: query batch exceeds device limits");
    const auto query_bytes = input.size() * sizeof(Eigen::Vector4f);
    const auto result_bytes = input.size() * sizeof(float);
    if (!queries || queries->capacity < query_bytes) {
      // Allocate both before replacing existing buffers.
      auto next_queries = context.buffer(query_bytes);
      auto next_results = context.buffer(result_bytes);
      queries = std::move(next_queries);
      results = std::move(next_results);
    }
    context.write(*queries, input.data(), query_bytes);
    output.resize(input.size());
    const std::array<const VulkanBuffer *, 4> buffers{
        nodes.get(), blocks.get(), queries.get(), results.get()};
    std::array<vk::DescriptorBufferInfo, 4> infos;
    std::array<vk::WriteDescriptorSet, 4> writes;
    std::array<vk::BufferMemoryBarrier, 4> barriers;
    for (uint32_t i = 0; i < buffers.size(); ++i) {
      infos[i] = vk::DescriptorBufferInfo(*buffers[i]->buffer, 0, buffers[i]->capacity);
      writes[i] = vk::WriteDescriptorSet(descriptor, i, 0, 1,
                                       vk::DescriptorType::eStorageBuffer, nullptr, &infos[i]);
      barriers[i] = vk::BufferMemoryBarrier(
          vk::AccessFlagBits::eHostWrite,
          i == 3 ? vk::AccessFlagBits::eShaderWrite : vk::AccessFlagBits::eShaderRead,
          VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED,
          *buffers[i]->buffer, 0, VK_WHOLE_SIZE);
    }
    context.device->updateDescriptorSets(writes, {});
    const auto command = *commands.front();
    command.reset();
    command.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
    command.pipelineBarrier(vk::PipelineStageFlagBits::eHost,
                            vk::PipelineStageFlagBits::eComputeShader, {}, {}, barriers, {});
    command.bindPipeline(vk::PipelineBindPoint::eCompute, *pipeline);
    command.bindDescriptorSets(vk::PipelineBindPoint::eCompute, *pipeline_layout, 0, descriptor, {});
    struct { uint32_t count; float limit; } push{static_cast<uint32_t>(input.size()), limit};
    command.pushConstants(*pipeline_layout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(push), &push);
    command.dispatch(static_cast<uint32_t>(groups), 1, 1);
    const vk::BufferMemoryBarrier readback(
        vk::AccessFlagBits::eShaderWrite, vk::AccessFlagBits::eHostRead,
        VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, *results->buffer, 0, VK_WHOLE_SIZE);
    command.pipelineBarrier(vk::PipelineStageFlagBits::eComputeShader,
                            vk::PipelineStageFlagBits::eHost, {}, {}, readback, {});
    command.end();
    context.submit(command);
    context.read(*results, output.data(), result_bytes);
  }
};

VulkanDistance::VulkanDistance(const MapIndex &index) : impl_(std::make_unique<Impl>(index)) {}
VulkanDistance::~VulkanDistance() = default;
void VulkanDistance::squaredDistances(const std::vector<Eigen::Vector4f> &queries, float limit,
                                     std::vector<float> &output) {
  impl_->run(queries, limit, output);
}
std::string VulkanDistance::deviceName() const { return impl_->context.properties.deviceName.data(); }
}  // namespace emcl3d::detail
