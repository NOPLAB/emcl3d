// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/particle_filter.hpp"
#include "map_fixture.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace {
TEST(Vulkan, BatchDistancesMatchScalarAcrossBoundsAndBufferReuse) {
  emcl3d::test::IrregularMapFile fixture;
  emcl3d::Map map("vulkan");
  map.load(fixture.path(), 0.01);
  std::vector<Eigen::Vector4f> queries;
  std::vector<float> result;
  for (size_t count : {1u, 257u, 17u, 1003u, 0u}) {
    queries.resize(count);
    for (size_t i = 0; i < count; ++i)
      queries[i] = Eigen::Vector4f(0.49f * (i % 19) - 1,
          0.43f * ((i * 7) % 23) - 1, 0.37f * ((i * 11) % 29) - 1, 1);
    for (float limit : {std::numeric_limits<float>::max(), 0.25f, 1.0f, 7.0f, 0.0f}) {
      SCOPED_TRACE(count);
      SCOPED_TRACE(limit);
      map.squaredDistances(queries, limit, result);
      ASSERT_EQ(result.size(), queries.size());
      for (size_t i = 0; i < count; ++i) {
        SCOPED_TRACE(i);
        ASSERT_FLOAT_EQ(std::sqrt(result[i]),
            map.distance(queries[i].head<3>().cast<double>(), limit));
      }
    }
  }
}
TEST(Vulkan, FilterUpdatesPreserveCpuEstimatesAndRandomSequence) {
  auto cpu_map = std::make_shared<emcl3d::Map>("cpu");
  auto gpu_map = std::make_shared<emcl3d::Map>("vulkan");
  cpu_map->load(TEST_MAP_PATH, 0.01);
  gpu_map->load(TEST_MAP_PATH, 0.01);
  const std::vector<Eigen::Vector3d> points{{0,0,0}, {2,0,0}, {0,3,0}, {0,0,4},
      {2,3,0}, {2,0,4}, {0,3,4}, {2,3,4}};
  for (bool reset : {false, true}) {
    SCOPED_TRACE(reset);
    emcl3d::Config config;
    config.num_particles = 50;
    config.num_threads = 2;
    config.min_valid_points = 4;
    config.translation_noise.setConstant(0.03);
    config.rotation_noise.setConstant(0.02);
    config.resample_ess_ratio = reset ? 0 : 1;
    config.alpha_threshold = reset ? 1 : 0;
    config.inlier_distance = reset ? 0.0001 : 0.5;
    config.reset_confirmation_updates = 2;
    emcl3d::ParticleFilter cpu(config, cpu_map), gpu(config, gpu_map);
    auto pose = emcl3d::Pose::Identity();
    pose.translation() = Eigen::Vector3d(0.1, 0.05, -0.03);
    const emcl3d::Matrix6 covariance = emcl3d::Matrix6::Identity() * 0.015;
    cpu.initialize(pose, covariance);
    gpu.initialize(pose, covariance);
    auto delta = emcl3d::Pose::Identity();
    delta.translation() = Eigen::Vector3d(0.01, -0.02, 0.005);
    delta.linear() = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()).toRotationMatrix();
    for (int update = 0; update < 4; ++update) {
      const auto a = cpu.update(delta, points), b = gpu.update(delta, points);
      ASSERT_TRUE(a.pose.matrix().isApprox(b.pose.matrix(), 1e-6));
      EXPECT_TRUE(a.covariance.isApprox(b.covariance, 1e-6));
      EXPECT_NEAR(a.ess, b.ess, 1e-4);
      EXPECT_DOUBLE_EQ(a.alpha, b.alpha);
      EXPECT_EQ(a.reset, b.reset);
      for (size_t i = 0; i < cpu.particles().size(); ++i) {
        EXPECT_TRUE(cpu.particles()[i].pose.matrix().isApprox(gpu.particles()[i].pose.matrix(), 1e-6));
        EXPECT_NEAR(cpu.particles()[i].weight, gpu.particles()[i].weight, 1e-6);
      }
    }
  }
}
TEST(Vulkan, CpuFloatSqrtBoundaryRetainsInliers) {
  auto map = std::make_shared<emcl3d::Map>("vulkan");
  map->load(TEST_MAP_PATH, 0.01);
  emcl3d::Config config;
  config.num_particles = 50;
  config.num_threads = 2;
  config.min_valid_points = 4;
  config.sigma_hit = 0.001;
  config.inlier_distance = std::nextafter(1.0, 2.0);
  const std::vector<Eigen::Vector3d> observation(4,
      {3, 3 + std::sqrt(std::numeric_limits<float>::epsilon()), 4});
  emcl3d::ParticleFilter filter(config, map);
  filter.initialize(emcl3d::Pose::Identity(), emcl3d::Matrix6::Zero());
  EXPECT_EQ(filter.update(emcl3d::Pose::Identity(), observation).alpha, 1.0);
}
}  // namespace
