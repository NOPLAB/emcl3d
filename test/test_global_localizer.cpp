// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/global_localizer.hpp"
#include "map_fixture.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace {
constexpr double pi = 3.14159265358979323846;
emcl3d::GlobalLocalizationConfig config() {
  emcl3d::GlobalLocalizationConfig c;
  c.free_cells_only = false;
  c.candidates = 1000;
  c.batch_size = 37;
  c.num_threads = 1;
  c.min_xy = Eigen::Vector2d(-1, -1);
  c.max_xy = Eigen::Vector2d(1, 1);
  c.inlier_distance = 0.3;
  c.min_inlier_ratio = 0.95;
  c.max_mean_squared_error = 0.001;
  return c;
}
void run(emcl3d::GlobalLocalizer &search, size_t batch) {
  while (!search.progress().complete) {
    const auto before = search.progress().evaluated;
    search.step();
    ASSERT_LE(search.progress().evaluated - before, batch);
  }
  EXPECT_EQ(search.progress().evaluated, search.progress().total);
}
TEST(GlobalLocalizer, RecoversTransformedIrregularScanIndependentOfBatches) {
  emcl3d::test::IrregularMapFile fixture;
  auto map = std::make_shared<emcl3d::Map>();
  map->load(fixture.path(), 0.01);
  auto c = config();
  c.roll = 0.2;
  c.pitch = 2.0;
  emcl3d::Pose truth = emcl3d::Pose::Identity();
  truth.translation() = Eigen::Vector3d(0.325, -0.275, 0);
  truth.linear() = (Eigen::AngleAxisd(pi / 8, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(c.pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(c.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  std::vector<Eigen::Vector3d> scan;
  for (const auto &p : fixture.points) scan.push_back(truth.inverse() * p.cast<double>());
  emcl3d::GlobalLocalizer search(map, c, scan);
  EXPECT_THROW(search.result(), std::logic_error);
  run(search, c.batch_size);
  ASSERT_TRUE(search.result().success) << search.result().reason << "\n" <<
      search.result().pose.matrix() << "\nMSE=" <<
      search.result().hypotheses.front().mean_squared_error;
  EXPECT_DOUBLE_EQ(search.progress().best_inlier_ratio, 1);
  EXPECT_LT((search.result().pose.translation() - truth.translation()).norm(), 0.03);
  EXPECT_LT(Eigen::AngleAxisd(search.result().pose.rotation() * truth.rotation().transpose()).angle(), 0.04);
  EXPECT_LT(search.result().covariance(0, 0), 0.02);
  EXPECT_LT(search.result().covariance(5, 5), 0.02);
  for (const auto &h : search.result().hypotheses) {
    EXPECT_GE(h.pose.translation().x(), -1);
    EXPECT_LE(h.pose.translation().x(), 1);
    EXPECT_GE(h.pose.translation().y(), -1);
    EXPECT_LE(h.pose.translation().y(), 1);
  }
  c.batch_size = 256;
  emcl3d::GlobalLocalizer batched(map, c, scan);
  run(batched, c.batch_size);
  EXPECT_EQ(batched.result().reason, search.result().reason);
  EXPECT_TRUE(batched.result().pose.matrix().isApprox(search.result().pose.matrix(), 1e-12));
  EXPECT_TRUE(batched.result().covariance.isApprox(search.result().covariance, 1e-12));
}
TEST(FreeSpaceGrid, RotatedOriginRejectsUnknownOccupiedAndFootprintOverlap) {
  emcl3d::FreeSpaceGrid grid;
  grid.resolution = 1;
  grid.width = grid.height = 5;
  grid.data.assign(25, 0);
  grid.data[2 * 5 + 3] = 100;
  grid.data[1 * 5 + 2] = -1;
  grid.origin.translation() = Eigen::Vector3d(10, 20, 0);
  grid.origin.linear() = Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  grid.validate();
  const auto world = [&](double x, double y) {
    return (grid.origin * Eigen::Vector3d(x, y, 0)).head<2>().eval();
  };
  EXPECT_TRUE(grid.allows(world(2.5, 2.5), 0.4));
  EXPECT_FALSE(grid.allows(world(2.5, 2.5), 0.6));
  EXPECT_FALSE(grid.allows(world(3.5, 2.5), 0));
  EXPECT_FALSE(grid.allows(world(2.5, 1.5), 0));
  EXPECT_FALSE(grid.allows(world(0.1, 0.1), 0.2));
  grid.data.pop_back();
  EXPECT_THROW(grid.validate(), std::invalid_argument);
}
TEST(GlobalLocalizer, RejectsAmbiguousAndPoorMatches) {
  auto map = std::make_shared<emcl3d::Map>();
  map->load(TEST_MAP_PATH, 0.01);
  auto c = config();
  c.min_xy = Eigen::Vector2d(0, 0);
  c.max_xy = Eigen::Vector2d(2, 3);
  c.candidates = 4096;
  c.min_valid_points = 4;
  c.min_inlier_ratio = 0.8;
  c.max_mean_squared_error = 0.05;
  c.mode_separation_xy = 0.5;
  std::vector<Eigen::Vector3d> scan = {{-1,-1.5,0}, {1,-1.5,0},
      {-1,1.5,0}, {1,1.5,0}, {-1,-1.5,4}, {1,-1.5,4}, {-1,1.5,4}, {1,1.5,4}};
  emcl3d::GlobalLocalizer ambiguous(map, c, scan);
  run(ambiguous, c.batch_size);
  EXPECT_FALSE(ambiguous.result().success);
  EXPECT_EQ(ambiguous.result().reason, "ambiguous_match");
  for (auto &p : scan) p.z() += 100;
  emcl3d::GlobalLocalizer poor(map, c, scan);
  run(poor, c.batch_size);
  EXPECT_FALSE(poor.result().success);
  EXPECT_EQ(poor.result().reason, "insufficient_match");
}
TEST(GlobalLocalizer, RequiresGridAndRejectsEmptyFreeSpace) {
  auto map = std::make_shared<emcl3d::Map>();
  map->load(TEST_MAP_PATH, 0.01);
  auto c = config();
  c.free_cells_only = true;
  c.min_valid_points = 1;
  std::vector<Eigen::Vector3d> scan = {{0,0,0}};
  EXPECT_THROW(emcl3d::GlobalLocalizer(map, c, scan), std::invalid_argument);
  emcl3d::FreeSpaceGrid grid;
  grid.resolution = 1;
  grid.width = grid.height = 4;
  grid.data.assign(16, -1);
  emcl3d::GlobalLocalizer search(map, c, scan, grid);
  run(search, c.batch_size);
  EXPECT_EQ(search.result().reason, "no_valid_candidates");
  c.batch_size = 0;
  EXPECT_THROW(emcl3d::GlobalLocalizer(map, c, scan, grid), std::invalid_argument);
}
}  // namespace
