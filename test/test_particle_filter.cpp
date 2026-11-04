// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/particle_filter.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
std::shared_ptr<emcl3d::Map> map() {
  auto value = std::make_shared<emcl3d::Map>();
  value->load(TEST_MAP_PATH, 0.01);
  return value;
}
std::vector<Eigen::Vector3d> points() {
  return {{0,0,0}, {2,0,0}, {0,3,0}, {0,0,4},
          {2,3,0}, {2,0,4}, {0,3,4}, {2,3,4}};
}
emcl3d::Config config() {
  emcl3d::Config c;
  c.num_particles = 100;
  c.num_threads = 1;
  c.min_valid_points = 4;
  c.translation_noise.setZero();
  c.rotation_noise.setZero();
  return c;
}
TEST(Map, DistanceBoundsAndMissingFile) {
  auto m = map();
  EXPECT_NEAR(m->distance({2,3,4}), 0, 1e-6);
  EXPECT_NEAR(m->distance({2,3,5}), 1, 1e-6);
  EXPECT_NEAR(m->distance({3,4,5}), std::sqrt(3.0), 1e-6);
  EXPECT_NEAR(m->distance({2,3,5}, std::nextafter(1.0f, 2.0f)), 1, 1e-6);
  EXPECT_TRUE(std::isinf(m->distance({2,3,5}, 0.25f)));
  EXPECT_TRUE(m->contains({1,1,1}, 0));
  EXPECT_FALSE(m->contains({20,1,1}, 0));
  EXPECT_THROW(m->load("nonexistent-emcl3d-map.pcd", 0.1), std::runtime_error);
}
TEST(Map, PackedSearchMatchesBruteForce) {
  std::unique_ptr<FILE, decltype(&std::fclose)> file(std::tmpfile(), &std::fclose);
  ASSERT_NE(file.get(), nullptr);
  std::vector<Eigen::Vector3f> grid;
  for (int x = 0; x < 8; ++x)
    for (int y = 0; y < 8; ++y)
      for (int z = 0; z < 8; ++z) {
        if (x == 7 && y == 7 && z == 7) continue;
        grid.emplace_back(x + 0.17f * ((y * 13 + z * 7) % 5),
            y + 0.13f * ((x * 11 + z * 3) % 7), z + 0.19f * ((x * 5 + y * 3) % 3));
      }
  std::fprintf(file.get(),
      "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
      "WIDTH %zu\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %zu\nDATA ascii\n",
      grid.size(), grid.size());
  for (const auto &p : grid) std::fprintf(file.get(), "%.9g %.9g %.9g\n", p.x(), p.y(), p.z());
  ASSERT_EQ(std::fflush(file.get()), 0);
  emcl3d::Map m;
  m.load("/proc/self/fd/" + std::to_string(::fileno(file.get())), 0.01);
  for (int i = 0; i < 256; ++i) {
    const Eigen::Vector3f query(0.49f * (i % 19) - 1,
        0.43f * ((i * 7) % 23) - 1, 0.37f * ((i * 11) % 29) - 1);
    float squared = std::numeric_limits<float>::max();
    for (const auto &point : grid) {
      const Eigen::Vector3f d = query - point;
      squared = std::min(squared, (d.x() * d.x() + d.y() * d.y()) + d.z() * d.z());
    }
    SCOPED_TRACE(i);
    ASSERT_FLOAT_EQ(m.distance(query.cast<double>()), std::sqrt(squared));
    EXPECT_FLOAT_EQ(m.distance(query.cast<double>(), squared + 1), std::sqrt(squared));
    EXPECT_TRUE(std::isinf(m.distance(query.cast<double>(), squared * 0.5f)));
  }
}
TEST(Filter, ExactPoseAndCovariance) {
  emcl3d::ParticleFilter f(config(), map());
  f.initialize(emcl3d::Pose::Identity(), emcl3d::Matrix6::Zero());
  const auto result = f.update(emcl3d::Pose::Identity(), points());
  EXPECT_TRUE(result.pose.matrix().isApprox(emcl3d::Pose::Identity().matrix(), 1e-12));
  EXPECT_NEAR(result.alpha, 1, 1e-12);
  EXPECT_NEAR(result.ess, 100, 1e-9);
  EXPECT_TRUE(result.covariance.isZero(1e-12));
  EXPECT_FALSE(result.reset);
}
TEST(Filter, BoundedLikelihoodMatchesGlobalSearch) {
  const auto m = map();
  auto observation = points();
  observation.push_back({1,1,1});
  observation.push_back({20,20,20});
  observation.push_back({-30,-30,-30});
  for (double sigma : {0.001, 0.3, 1.0, 1e200}) {
    SCOPED_TRACE(sigma);
    auto c = config();
    c.num_particles = 8;
    c.sigma_hit = sigma;
    c.resample_ess_ratio = 0;
    c.alpha_threshold = 0;
    emcl3d::ParticleFilter filter(c, m);
    filter.initialize(emcl3d::Pose::Identity(), emcl3d::Matrix6::Identity() * 0.015);
    std::vector<double> logs;
    int good = 0;
    for (const auto &p : filter.particles()) {
      double score = 0;
      int inliers = 0;
      for (const auto &point : observation) {
        const double d = m->distance(p.pose * point);
        score += std::log(c.z_hit * std::exp(-0.5 * d * d /
            (c.sigma_hit * c.sigma_hit)) + c.z_random);
        inliers += d < c.inlier_distance;
      }
      good += double(inliers) / observation.size() >= c.min_inlier_ratio;
      logs.push_back(std::log(p.weight) + c.likelihood_temperature * score / observation.size());
    }
    const double maximum = *std::max_element(logs.begin(), logs.end());
    double sum = 0;
    for (auto &value : logs) { value = std::exp(value - maximum); sum += value; }
    const auto result = filter.update(emcl3d::Pose::Identity(), observation);
    EXPECT_DOUBLE_EQ(result.alpha, double(good) / c.num_particles);
    for (size_t i = 0; i < logs.size(); ++i)
      EXPECT_DOUBLE_EQ(filter.particles()[i].weight, logs[i] / sum);
  }
}
TEST(Filter, FloatSqrtBoundaryRetainsInliers) {
  auto c = config();
  c.sigma_hit = 0.001;
  c.inlier_distance = std::nextafter(1.0, 2.0);
  const auto m = map();
  const std::vector<Eigen::Vector3d> observation(4,
      {3, 3 + std::sqrt(std::numeric_limits<float>::epsilon()), 4});
  ASSERT_EQ(m->distance(observation.front()), 1.0);
  emcl3d::ParticleFilter filter(c, m);
  filter.initialize(emcl3d::Pose::Identity(), emcl3d::Matrix6::Zero());
  EXPECT_EQ(filter.update(emcl3d::Pose::Identity(), observation).alpha, 1.0);
}
TEST(Filter, ThreeDimensionalMotionComposition) {
  emcl3d::ParticleFilter f(config(), map());
  emcl3d::Pose initial = emcl3d::Pose::Identity();
  initial.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  initial.translation() = Eigen::Vector3d(0.1,0.2,0.3);
  f.initialize(initial, emcl3d::Matrix6::Zero());
  emcl3d::Pose delta = emcl3d::Pose::Identity();
  delta.translation() = Eigen::Vector3d(0.1,0.2,0.3);
  delta.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  auto result = f.update(delta, points());
  EXPECT_TRUE(result.pose.matrix().isApprox((initial * delta).matrix(), 1e-10));
}
TEST(Filter, DeterministicInitializationAndInvalidCovariance) {
  emcl3d::ParticleFilter a(config(), map()), b(config(), map());
  emcl3d::Matrix6 covariance = emcl3d::Matrix6::Identity() * 0.01;
  a.initialize(emcl3d::Pose::Identity(), covariance);
  b.initialize(emcl3d::Pose::Identity(), covariance);
  for (size_t i=0; i<a.particles().size(); ++i)
    EXPECT_TRUE(a.particles()[i].pose.matrix().isApprox(b.particles()[i].pose.matrix()));
  covariance(0,0) = -1;
  EXPECT_THROW(a.initialize(emcl3d::Pose::Identity(), covariance), std::invalid_argument);
  EXPECT_THROW(a.update(emcl3d::Pose::Identity(), {}), std::invalid_argument);
}
TEST(Filter, ResetRequiresConsecutiveSupportedFailures) {
  auto c = config();
  c.inlier_distance = 0.01;
  c.reset_confirmation_updates = 2;
  emcl3d::ParticleFilter f(c, map());
  auto pose = emcl3d::Pose::Identity();
  // Shift along one axis: half the corners remain inside the map bounds,
  // while no endpoint is an inlier. This is a supported localization failure.
  pose.translation() = Eigen::Vector3d(0.1,0,0);
  f.initialize(pose, emcl3d::Matrix6::Zero());
  EXPECT_FALSE(f.update(emcl3d::Pose::Identity(), points()).reset);
  EXPECT_TRUE(f.update(emcl3d::Pose::Identity(), points()).reset);
  for (const auto &p : f.particles())
    EXPECT_TRUE(((p.pose.translation()-pose.translation()).array().abs() <=
                 c.expansion_translation.array()+1e-12).all());
  pose.translation() = Eigen::Vector3d(100,100,100);
  f.initialize(pose, emcl3d::Matrix6::Zero());
  EXPECT_FALSE(f.update(emcl3d::Pose::Identity(), points()).reset);
  EXPECT_FALSE(f.update(emcl3d::Pose::Identity(), points()).reset);
}
TEST(Filter, ObservationReweightsAndResamplingRestoresUniformWeights) {
  auto c = config();
  c.num_particles = 600;
  c.sigma_hit = 0.1;
  c.likelihood_temperature = 40;
  c.resample_ess_ratio = 1;
  c.alpha_threshold = 0;
  emcl3d::ParticleFilter f(c, map());
  emcl3d::Matrix6 covariance = emcl3d::Matrix6::Zero();
  covariance(0,0) = 0.25;
  f.initialize(emcl3d::Pose::Identity(), covariance);
  const auto result = f.update(emcl3d::Pose::Identity(), points());
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
TEST(Filter, ThreadCountsPreserveLikelihoodResetAndRandomSequence) {
  const auto m = map();
  for (int threads : {2, 4, 0, -3}) {
    for (bool reset : {false, true}) {
      SCOPED_TRACE(threads);
      SCOPED_TRACE(reset);
      auto c = config();
      c.num_particles = 137;
      c.translation_noise.setConstant(0.03);
      c.rotation_noise.setConstant(0.02);
      c.resample_ess_ratio = reset ? 0 : 1;
      c.alpha_threshold = reset ? 1 : 0;
      c.inlier_distance = reset ? 0.0001 : 0.5;
      c.reset_confirmation_updates = 2;
      emcl3d::ParticleFilter serial(c, m);
      c.num_threads = threads;
      emcl3d::ParticleFilter parallel(c, m);
      auto pose = emcl3d::Pose::Identity();
      pose.translation() = Eigen::Vector3d(0.1, 0.05, -0.03);
      const emcl3d::Matrix6 covariance = emcl3d::Matrix6::Identity() * 0.015;
      serial.initialize(pose, covariance);
      parallel.initialize(pose, covariance);
      ASSERT_FALSE(serial.particles()[0].pose.matrix().isApprox(
        serial.particles()[1].pose.matrix()));
      auto delta = emcl3d::Pose::Identity();
      delta.translation() = Eigen::Vector3d(0.01, -0.02, 0.005);
      delta.linear() = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()).toRotationMatrix();
      bool saw_reset = false, saw_reweighting = false;
      for (int update = 0; update < 4; ++update) {
        const auto a = serial.update(delta, points());
        const auto b = parallel.update(delta, points());
        EXPECT_TRUE((a.pose.matrix().array() == b.pose.matrix().array()).all());
        EXPECT_TRUE((a.covariance.array() == b.covariance.array()).all());
        EXPECT_EQ(a.alpha, b.alpha);
        EXPECT_EQ(a.ess, b.ess);
        EXPECT_EQ(a.reset, b.reset);
        saw_reset |= a.reset;
        saw_reweighting |= a.ess < c.num_particles - 1;
        for (size_t i = 0; i < serial.particles().size(); ++i) {
          const auto &x = serial.particles()[i];
          const auto &y = parallel.particles()[i];
          EXPECT_TRUE((x.pose.matrix().array() == y.pose.matrix().array()).all());
          EXPECT_EQ(x.weight, y.weight);
          if (!reset) EXPECT_DOUBLE_EQ(x.weight, 1.0 / c.num_particles);
        }
      }
      EXPECT_EQ(saw_reset, reset);
      EXPECT_TRUE(saw_reweighting);
    }
  }
}
}
