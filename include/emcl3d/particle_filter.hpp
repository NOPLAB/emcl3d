// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace emcl3d {
using Pose = Eigen::Isometry3d;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
struct Particle {
  Pose pose = Pose::Identity();
  double weight = 0;
};
struct Config {
  int num_particles = 500;
  unsigned random_seed = 1;
  double resample_ess_ratio = 0.5;
  double sigma_hit = 0.3, z_hit = 0.95, z_random = 0.05;
  double likelihood_temperature = 10;
  double inlier_distance = 0.5, min_inlier_ratio = 0.3, alpha_threshold = 0.5;
  int reset_confirmation_updates = 3, min_valid_points = 20;
  Eigen::Vector3d translation_noise = Eigen::Vector3d::Constant(0.1);
  Eigen::Vector3d rotation_noise = Eigen::Vector3d::Constant(0.1);
  Eigen::Vector3d expansion_translation = Eigen::Vector3d::Constant(0.2);
  Eigen::Vector3d expansion_rotation = Eigen::Vector3d::Constant(0.1);
};
class Map {
public:
  Map();
  ~Map();
  void load(const std::string &path, double voxel_size);
  double distance(const Eigen::Vector3d &point) const;
  bool contains(const Eigen::Vector3d &point, double margin) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
struct Estimate {
  Pose pose = Pose::Identity();
  Matrix6 covariance = Matrix6::Zero();
  double alpha = 0, ess = 0;
  bool reset = false;
};
class ParticleFilter {
public:
  ParticleFilter(Config config, std::shared_ptr<const Map> map);
  void initialize(const Pose &pose, const Matrix6 &covariance);
  Estimate update(const Pose &odom_delta, const std::vector<Eigen::Vector3d> &points_base);
  const std::vector<Particle> &particles() const { return particles_; }
private:
  double evaluate(const std::vector<Eigen::Vector3d> &points);
  Estimate estimate() const;
  void resample();
  Config config_;
  std::shared_ptr<const Map> map_;
  std::vector<Particle> particles_;
  std::mt19937 random_;
  int bad_updates_ = 0;
};
}  // namespace emcl3d
