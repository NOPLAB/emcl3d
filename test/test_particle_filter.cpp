// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "mcl3d/particle_filter.hpp"
#include <gtest/gtest.h>

namespace {
std::shared_ptr<mcl3d::Map> map() {
  auto value = std::make_shared<mcl3d::Map>();
  value->load(TEST_MAP_PATH, 0.01);
  return value;
}
std::vector<Eigen::Vector3d> points() {
  return {{0,0,0}, {2,0,0}, {0,3,0}, {0,0,4},
          {2,3,0}, {2,0,4}, {0,3,4}, {2,3,4}};
}
mcl3d::Config config() {
  mcl3d::Config c;
  c.num_particles = 100;
  c.min_valid_points = 4;
  c.translation_noise.setZero();
  c.rotation_noise.setZero();
  return c;
}
TEST(Map, DistanceBoundsAndMissingFile) {
  auto m = map();
  EXPECT_NEAR(m->distance({2,3,4}), 0, 1e-6);
  EXPECT_NEAR(m->distance({2,3,5}), 1, 1e-6);
  EXPECT_TRUE(m->contains({1,1,1}, 0));
  EXPECT_FALSE(m->contains({20,1,1}, 0));
  EXPECT_THROW(m->load("nonexistent-mcl3d-map.pcd", 0.1), std::runtime_error);
}
TEST(Filter, ExactPoseAndCovariance) {
  mcl3d::ParticleFilter f(config(), map());
  f.initialize(mcl3d::Pose::Identity(), mcl3d::Matrix6::Zero());
  const auto result = f.update(mcl3d::Pose::Identity(), points());
  EXPECT_TRUE(result.pose.matrix().isApprox(mcl3d::Pose::Identity().matrix(), 1e-12));
  EXPECT_NEAR(result.alpha, 1, 1e-12);
  EXPECT_NEAR(result.ess, 100, 1e-9);
  EXPECT_TRUE(result.covariance.isZero(1e-12));
  EXPECT_FALSE(result.reset);
}
TEST(Filter, ThreeDimensionalMotionComposition) {
  mcl3d::ParticleFilter f(config(), map());
  mcl3d::Pose initial = mcl3d::Pose::Identity();
  initial.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  initial.translation() = Eigen::Vector3d(0.1,0.2,0.3);
  f.initialize(initial, mcl3d::Matrix6::Zero());
  mcl3d::Pose delta = mcl3d::Pose::Identity();
  delta.translation() = Eigen::Vector3d(0.1,0.2,0.3);
  delta.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  auto result = f.update(delta, points());
  EXPECT_TRUE(result.pose.matrix().isApprox((initial * delta).matrix(), 1e-10));
}
TEST(Filter, DeterministicInitializationAndInvalidCovariance) {
  mcl3d::ParticleFilter a(config(), map()), b(config(), map());
  mcl3d::Matrix6 covariance = mcl3d::Matrix6::Identity() * 0.01;
  a.initialize(mcl3d::Pose::Identity(), covariance);
  b.initialize(mcl3d::Pose::Identity(), covariance);
  for (size_t i=0; i<a.particles().size(); ++i)
    EXPECT_TRUE(a.particles()[i].pose.matrix().isApprox(b.particles()[i].pose.matrix()));
  covariance(0,0) = -1;
  EXPECT_THROW(a.initialize(mcl3d::Pose::Identity(), covariance), std::invalid_argument);
  EXPECT_THROW(a.update(mcl3d::Pose::Identity(), {}), std::invalid_argument);
}
TEST(Filter, ResetRequiresConsecutiveSupportedFailures) {
  auto c = config();
  c.inlier_distance = 0.01;
  c.reset_confirmation_updates = 2;
  mcl3d::ParticleFilter f(c, map());
  auto pose = mcl3d::Pose::Identity();
  // Shift along one axis: half the corners remain inside the map bounds,
  // while no endpoint is an inlier. This is a supported localization failure.
  pose.translation() = Eigen::Vector3d(0.1,0,0);
  f.initialize(pose, mcl3d::Matrix6::Zero());
  EXPECT_FALSE(f.update(mcl3d::Pose::Identity(), points()).reset);
  EXPECT_TRUE(f.update(mcl3d::Pose::Identity(), points()).reset);
  for (const auto &p : f.particles())
    EXPECT_TRUE(((p.pose.translation()-pose.translation()).array().abs() <=
                 c.expansion_translation.array()+1e-12).all());
  pose.translation() = Eigen::Vector3d(100,100,100);
  f.initialize(pose, mcl3d::Matrix6::Zero());
  EXPECT_FALSE(f.update(mcl3d::Pose::Identity(), points()).reset);
  EXPECT_FALSE(f.update(mcl3d::Pose::Identity(), points()).reset);
}
TEST(Filter, ObservationReweightsAndResamplingRestoresUniformWeights) {
  auto c = config();
  c.num_particles = 600;
  c.sigma_hit = 0.1;
  c.likelihood_temperature = 40;
  c.resample_ess_ratio = 1;
  c.alpha_threshold = 0;
  mcl3d::ParticleFilter f(c, map());
  mcl3d::Matrix6 covariance = mcl3d::Matrix6::Zero();
  covariance(0,0) = 0.25;
  f.initialize(mcl3d::Pose::Identity(), covariance);
  const auto result = f.update(mcl3d::Pose::Identity(), points());
  EXPECT_LT(result.ess, c.num_particles * 0.5);
  EXPECT_LT(std::abs(result.pose.translation().x()), 0.1);
  EXPECT_LT(result.covariance(0,0), 0.025);
  double total = 0;
  for (const auto &p : f.particles()) {
    EXPECT_NEAR(p.weight, 1.0 / c.num_particles, 1e-12);
    total += p.weight;
  }
  EXPECT_NEAR(total, 1, 1e-12);
}
}
