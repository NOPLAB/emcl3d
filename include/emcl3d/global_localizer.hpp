// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#pragma once
#include "emcl3d/particle_filter.hpp"
#include <cstdint>
#include <optional>

namespace emcl3d {
struct GlobalLocalizationConfig {
  size_t candidates = 100000, batch_size = 256, max_eval_points = 2000, num_hypotheses = 10;
  int refinement_levels = 3, min_valid_points = 20, num_threads = 0;
  std::optional<Eigen::Vector2d> min_xy, max_xy;
  double z = 0, roll = 0, pitch = 0;
  double yaw_min = -3.14159265358979323846, yaw_max = 3.14159265358979323846;
  double inlier_distance = 0.5, min_inlier_ratio = 0.6, max_mean_squared_error = 0.15;
  double sigma_hit = 0.3, z_hit = 0.95, z_random = 0.05;
  double ambiguity_score_margin = 0.02, mode_separation_xy = 1, mode_separation_yaw = 0.5;
  double footprint_radius = 0;
  double max_position_stddev = 0.5, max_yaw_stddev = 0.2;
  unsigned seed = 1;
  bool free_cells_only = true;
};
struct FreeSpaceGrid {
  double resolution = 0;
  size_t width = 0, height = 0;
  Pose origin = Pose::Identity();
  std::vector<int8_t> data;
  int occupancy_threshold = 50;
  void validate() const;
  bool allows(const Eigen::Vector2d &xy, double radius) const;
};
struct GlobalHypothesis {
  Pose pose = Pose::Identity();
  // Mean normalized Gaussian mixture; larger is better, range [z_random/(z_hit+z_random),1].
  // MSE alone is truncated at inlier_distance^2.
  double score = 0, inlier_ratio = 0, mean_squared_error = 0;
};
struct GlobalLocalizationResult {
  bool success = false;
  std::string reason;
  Pose pose = Pose::Identity();
  Matrix6 covariance = Matrix6::Zero();
  std::vector<GlobalHypothesis> hypotheses;
};
struct GlobalLocalizationProgress {
  size_t evaluated = 0, total = 0;
  int refinement_level = 0;
  double best_inlier_ratio = 0;
  bool complete = false;
};
// Call step sequentially; each call scores at most batch_size poses.
class GlobalLocalizer {
public:
  GlobalLocalizer(std::shared_ptr<const Map> map, GlobalLocalizationConfig config,
      std::vector<Eigen::Vector3d> points, std::optional<FreeSpaceGrid> grid = std::nullopt);
  bool step();
  const GlobalLocalizationProgress &progress() const { return progress_; }
  const GlobalLocalizationResult &result() const;
private:
  Pose pose(const Eigen::Vector3d &xy_yaw) const;
  bool allowed(const Eigen::Vector3d &xy_yaw) const;
  void retain(const GlobalHypothesis &hypothesis);
  void finishLevel();
  std::shared_ptr<const Map> map_;
  GlobalLocalizationConfig config_;
  std::vector<Eigen::Vector3d> points_, pending_;
  std::optional<FreeSpaceGrid> grid_;
  Eigen::Vector2d minimum_, maximum_;
  Eigen::Vector3d spacing_;
  Eigen::Vector3d scan_center_ = Eigen::Vector3d::Zero();
  size_t cursor_ = 0, nx_ = 0, nyaw_ = 0;
  std::vector<GlobalHypothesis> best_, local_samples_;
  GlobalLocalizationProgress progress_;
  GlobalLocalizationResult result_;
};
}  // namespace emcl3d
