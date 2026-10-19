// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/particle_filter.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace emcl3d {
namespace {
Eigen::Matrix3d rotation(const Eigen::Vector3d &v) {
  const double angle = v.norm();
  return angle < 1e-12 ? Eigen::Matrix3d::Identity()
                      : Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}
}
ParticleFilter::ParticleFilter(Config config, std::shared_ptr<const Map> map)
    : config_(std::move(config)), map_(std::move(map)), random_(config_.random_seed) {
  if (!map_ || config_.num_particles < 2 || config_.min_valid_points < 1 ||
      config_.reset_confirmation_updates < 1)
    throw std::invalid_argument("Invalid particle filter configuration");
  for (double value : {config_.resample_ess_ratio, config_.min_inlier_ratio,
                       config_.alpha_threshold})
    if (!std::isfinite(value) || value < 0 || value > 1)
      throw std::invalid_argument("Particle filter ratios must be in [0,1]");
  for (double value : {config_.sigma_hit, config_.z_hit, config_.z_random,
                       config_.likelihood_temperature, config_.inlier_distance})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Likelihood parameters must be positive");
  for (const auto &v : {config_.translation_noise, config_.rotation_noise,
                        config_.expansion_translation, config_.expansion_rotation})
    if (!v.allFinite() || (v.array() < 0).any())
      throw std::invalid_argument("Noise and expansion limits must be nonnegative");
}
void ParticleFilter::initialize(const Pose &pose, const Matrix6 &covariance) {
  if (!pose.matrix().allFinite() || !covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-9))
    throw std::invalid_argument("Invalid initial pose or covariance");
  Eigen::SelfAdjointEigenSolver<Matrix6> solver(covariance);
  if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-9)
    throw std::invalid_argument("Initial covariance must be positive semidefinite");
  const Matrix6 factor = solver.eigenvectors() *
      solver.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
  std::normal_distribution<double> normal;
  particles_.resize(config_.num_particles);
  for (auto &p : particles_) {
    Eigen::Matrix<double, 6, 1> sample;
    for (int j = 0; j < 6; ++j) sample[j] = normal(random_);
    sample = (factor * sample).eval();
    p.pose = pose;
    p.pose.translation() += sample.head<3>();
    // Initial covariance is expressed in fixed map axes.
    p.pose.linear() = rotation(sample.tail<3>()) * pose.rotation();
    p.weight = 1.0 / particles_.size();
  }
  bad_updates_ = 0;
}
double ParticleFilter::evaluate(const std::vector<Eigen::Vector3d> &points) {
  std::vector<double> logs;
  int good = 0;
  for (const auto &p : particles_) {
    double score = 0;
    int inliers = 0;
    for (const auto &point : points) {
      const double d = map_->distance(p.pose * point);
      inliers += d < config_.inlier_distance;
      score += std::log(config_.z_hit *
          std::exp(-0.5 * d * d / (config_.sigma_hit * config_.sigma_hit)) +
          config_.z_random);
    }
    good += double(inliers) / points.size() >= config_.min_inlier_ratio;
    logs.push_back(std::log(std::max(p.weight, std::numeric_limits<double>::min())) +
                   config_.likelihood_temperature * score / points.size());
  }
  const double maximum = *std::max_element(logs.begin(), logs.end());
  double sum = 0;
  for (double &value : logs) { value = std::exp(value - maximum); sum += value; }
  for (size_t i = 0; i < particles_.size(); ++i) particles_[i].weight = logs[i] / sum;
  return double(good) / particles_.size();
}
Estimate ParticleFilter::estimate() const {
  Estimate result;
  Eigen::Matrix4d accumulator = Eigen::Matrix4d::Zero();
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  double squared_weights = 0;
  for (const auto &p : particles_) {
    mean += p.weight * p.pose.translation();
    const Eigen::Vector4d q = Eigen::Quaterniond(p.pose.rotation()).coeffs();
    accumulator += p.weight * q * q.transpose();
    squared_weights += p.weight * p.weight;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(accumulator);
  result.pose.linear() = Eigen::Quaterniond(solver.eigenvectors().col(3)).toRotationMatrix();
  result.pose.translation() = mean;
  for (const auto &p : particles_) {
    Eigen::Matrix<double, 6, 1> residual;
    residual.head<3>() = p.pose.translation() - mean;
    const Eigen::AngleAxisd angle(p.pose.rotation() * result.pose.rotation().transpose());
    residual.tail<3>() = angle.angle() * angle.axis();
    result.covariance += p.weight * residual * residual.transpose();
  }
  result.ess = 1.0 / squared_weights;
  return result;
}
void ParticleFilter::resample() {
  const auto old = particles_;
  const double step = 1.0 / old.size();
  std::uniform_real_distribution<double> uniform(0, step);
  const double start = uniform(random_);
  size_t index = 0;
  double cumulative = old[0].weight;
  for (size_t i = 0; i < old.size(); ++i) {
    while (index + 1 < old.size() && start + i * step > cumulative)
      cumulative += old[++index].weight;
    particles_[i] = old[index];
    particles_[i].weight = step;
  }
}
Estimate ParticleFilter::update(
    const Pose &delta, const std::vector<Eigen::Vector3d> &points) {
  if (particles_.empty()) throw std::logic_error("Filter is not initialized");
  if (!delta.matrix().allFinite() || points.size() < size_t(config_.min_valid_points) ||
      std::any_of(points.begin(), points.end(), [](const auto &p) { return !p.allFinite(); }))
    throw std::invalid_argument("Invalid observation or odometry");
  std::normal_distribution<double> normal;
  const double scale = std::sqrt(delta.translation().norm() +
                                 std::abs(Eigen::AngleAxisd(delta.rotation()).angle()));
  for (auto &p : particles_) {
    Pose noise = Pose::Identity();
    Eigen::Vector3d angle;
    for (int j = 0; j < 3; ++j) {
      noise.translation()[j] = normal(random_) * config_.translation_noise[j] * scale;
      angle[j] = normal(random_) * config_.rotation_noise[j] * scale;
    }
    noise.linear() = rotation(angle);
    p.pose = p.pose * delta * noise;
  }
  double alpha = evaluate(points);
  bool supported = false;
  for (const auto &p : particles_) {
    size_t inside = 0;
    for (const auto &point : points)
      inside += map_->contains(p.pose * point, config_.inlier_distance);
    supported |= double(inside) / points.size() >= config_.min_inlier_ratio;
  }
  bad_updates_ = supported && alpha < config_.alpha_threshold ? bad_updates_ + 1 : 0;
  const bool reset = bad_updates_ >= config_.reset_confirmation_updates;
  if (reset) {
    std::uniform_real_distribution<double> uniform(-1, 1);
    for (auto &p : particles_) {
      Eigen::Vector3d angle;
      for (int j = 0; j < 3; ++j) {
        p.pose.translation()[j] += uniform(random_) * config_.expansion_translation[j];
        angle[j] = uniform(random_) * config_.expansion_rotation[j];
      }
      p.pose.linear() = rotation(angle) * p.pose.rotation();
      p.weight = 1.0 / particles_.size();
    }
    alpha = evaluate(points);
    bad_updates_ = 0;
  }
  auto result = estimate();
  result.alpha = alpha;
  result.reset = reset;
  if (result.ess < config_.resample_ess_ratio * particles_.size()) resample();
  return result;
}
}  // namespace emcl3d
