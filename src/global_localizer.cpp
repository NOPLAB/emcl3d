// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include "emcl3d/global_localizer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace emcl3d {
namespace {
constexpr double pi = 3.14159265358979323846;
double yaw(const Pose &p, const GlobalLocalizationConfig &c) {
  const Eigen::Vector3d north =
      p.linear().col(1) * std::cos(c.roll) - p.linear().col(2) * std::sin(c.roll);
  return std::atan2(-north.x(), north.y());
}
double angle(double a) { return std::remainder(a, 2 * pi); }
bool sameMode(const GlobalHypothesis &a, const GlobalHypothesis &b,
    const GlobalLocalizationConfig &c) {
  return (a.pose.translation() - b.pose.translation()).head<2>().norm() < c.mode_separation_xy &&
      std::abs(angle(yaw(a.pose, c) - yaw(b.pose, c))) < c.mode_separation_yaw;
}
}
void FreeSpaceGrid::validate() const {
  if (!std::isfinite(resolution) || resolution <= 0 || !width || !height ||
      width > size_t(std::numeric_limits<int>::max()) ||
      height > size_t(std::numeric_limits<int>::max()) ||
      width > std::numeric_limits<size_t>::max() / height || data.size() != width * height ||
      occupancy_threshold < 1 || occupancy_threshold > 100 ||
      !origin.matrix().allFinite() || !origin.linear().isUnitary(1e-6) ||
      std::abs(origin.linear().determinant() - 1) > 1e-6 ||
      std::abs(origin.linear()(2, 2) - 1) > 1e-6 ||
      std::any_of(data.begin(), data.end(), [](int8_t v) { return v < -1 || v > 100; }))
    throw std::invalid_argument("invalid free-space grid");
}
bool FreeSpaceGrid::allows(const Eigen::Vector2d &xy, double radius) const {
  const auto p = origin.inverse() * Eigen::Vector3d(xy.x(), xy.y(), origin.translation().z());
  const double x = p.x() / resolution, y = p.y() / resolution, r = radius / resolution;
  if (x - r < 0 || y - r < 0 || x + r >= width || y + r >= height) return false;
  for (int j = int(std::floor(y - r)); j <= int(std::floor(y + r)); ++j)
    for (int i = int(std::floor(x - r)); i <= int(std::floor(x + r)); ++i) {
      const double dx = std::max({double(i) - x, 0.0, x - (i + 1)});
      const double dy = std::max({double(j) - y, 0.0, y - (j + 1)});
      if (dx * dx + dy * dy > r * r) continue;
      const auto v = data[size_t(j) * width + size_t(i)];
      if (v < 0 || v >= occupancy_threshold) return false;
    }
  return true;
}
GlobalLocalizer::GlobalLocalizer(std::shared_ptr<const Map> map, GlobalLocalizationConfig c,
    std::vector<Eigen::Vector3d> points, std::optional<FreeSpaceGrid> grid)
    : map_(std::move(map)), config_(c), points_(std::move(points)), grid_(std::move(grid)) {
  const double values[] = {c.z, c.roll, c.pitch, c.yaw_min, c.yaw_max, c.inlier_distance,
      c.min_inlier_ratio, c.max_mean_squared_error, c.ambiguity_score_margin,
      c.mode_separation_xy, c.mode_separation_yaw, c.footprint_radius,
      c.max_position_stddev, c.max_yaw_stddev, c.sigma_hit, c.z_hit, c.z_random};
  if (!map_ || !std::all_of(std::begin(values), std::end(values),
          [](double v) { return std::isfinite(v); }) ||
      !c.candidates || !c.batch_size || !c.max_eval_points || c.num_hypotheses < 2 ||
      c.refinement_levels < 0 || c.refinement_levels > 20 || c.min_valid_points < 1 ||
      c.inlier_distance <= 0 || c.inlier_distance > std::sqrt(std::numeric_limits<float>::max()) ||
      c.yaw_min >= c.yaw_max || c.yaw_max - c.yaw_min > 2 * pi + 1e-9 ||
      c.min_inlier_ratio < 0 || c.min_inlier_ratio > 1 || c.max_mean_squared_error < 0 ||
      c.ambiguity_score_margin < 0 || c.mode_separation_xy <= 0 ||
      c.mode_separation_yaw <= 0 || c.footprint_radius < 0 ||
      c.sigma_hit <= 0 || c.z_hit <= 0 || c.z_random < 0 ||
      !std::isfinite(c.z_hit + c.z_random) ||
      c.max_position_stddev <= 0 || c.max_yaw_stddev <= 0)
    throw std::invalid_argument("invalid global localization configuration");
  if (c.free_cells_only && !grid_) throw std::invalid_argument("free_cells_only requires a grid");
  if (grid_) grid_->validate();
  const auto bounds = map_->bounds();
  minimum_ = c.min_xy.value_or(bounds.first.head<2>());
  maximum_ = c.max_xy.value_or(bounds.second.head<2>());
  if (!minimum_.allFinite() || !maximum_.allFinite() ||
      (minimum_.array() > maximum_.array()).any())
    throw std::invalid_argument("invalid global localization bounds");
  if (points_.size() < size_t(c.min_valid_points) || points_.size() > c.max_eval_points ||
      !std::all_of(points_.begin(), points_.end(), [](const auto &p) { return p.allFinite(); }))
    throw std::invalid_argument("invalid global localization points");
  for (const auto &point : points_) scan_center_ += point / double(points_.size());
  nx_ = std::max(size_t(1), size_t(std::cbrt(double(c.candidates))));
  nyaw_ = (c.candidates + nx_ * nx_ - 1) / (nx_ * nx_);
  spacing_ = {(maximum_.x() - minimum_.x()) / nx_,
      (maximum_.y() - minimum_.y()) / nx_, (c.yaw_max - c.yaw_min) / nyaw_};
  progress_.total = c.candidates;
}
Pose GlobalLocalizer::pose(const Eigen::Vector3d &v) const {
  Pose p = Pose::Identity();
  p.translation() = Eigen::Vector3d(v.x(), v.y(), config_.z);
  p.linear() = (Eigen::AngleAxisd(v.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(config_.pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(config_.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return p;
}
bool GlobalLocalizer::allowed(const Eigen::Vector3d &v) const {
  return (v.head<2>().array() >= minimum_.array()).all() &&
      (v.head<2>().array() <= maximum_.array()).all() &&
      v.z() >= config_.yaw_min && v.z() < config_.yaw_max &&
      (!config_.free_cells_only || grid_->allows(v.head<2>(), config_.footprint_radius));
}
void GlobalLocalizer::retain(const GlobalHypothesis &h) {
  for (const auto &b : best_)
    if (sameMode(h, b, config_) && b.score >= h.score) return;
  best_.erase(std::remove_if(best_.begin(), best_.end(), [&](const auto &b) {
    return sameMode(h, b, config_);
  }), best_.end());
  best_.push_back(h);
  std::stable_sort(best_.begin(), best_.end(), [](const auto &a, const auto &b) {
    return a.score > b.score;
  });
  if (best_.size() > config_.num_hypotheses) best_.pop_back();
}
bool GlobalLocalizer::step() {
  if (progress_.complete) return true;
  const size_t count = progress_.refinement_level == 0 ? config_.candidates : pending_.size();
  std::vector<Pose> poses;
  std::vector<Eigen::Vector4f> queries;
  const size_t attempted = std::min(config_.batch_size, count - cursor_);
  const size_t end = cursor_ + attempted;
  for (; cursor_ < end; ++cursor_) {
    Eigen::Vector3d v;
    if (progress_.refinement_level == 0) {
      const size_t i = (cursor_ + config_.seed % config_.candidates) % config_.candidates;
      v = {minimum_.x() + (i % nx_ + 0.5) * spacing_.x(),
          minimum_.y() + ((i / nx_) % nx_ + 0.5) * spacing_.y(),
          config_.yaw_min + (i / (nx_ * nx_) + 0.5) * spacing_.z()};
    } else v = pending_[cursor_];
    if (!allowed(v)) continue;
    poses.push_back(pose(v));
    for (const auto &p : points_) {
      const Eigen::Vector3d q = poses.back() * p;
      queries.emplace_back(float(q.x()), float(q.y()), float(q.z()), 0);
    }
  }
  progress_.evaluated += attempted;
  // Count attempted candidates, including those removed by the free-space mask.
  std::vector<float> distances;
  const double limit = config_.inlier_distance * config_.inlier_distance;
  const double radius = std::max(config_.inlier_distance, config_.sigma_hit * std::sqrt(
      std::max(0.0, -2 * std::log(std::numeric_limits<double>::epsilon() *
          config_.z_random / (8 * config_.z_hit)))));
  float squared_limit = std::nextafter(float(radius * radius),
      std::numeric_limits<float>::infinity());
  const double omitted = std::sqrt(squared_limit);
  if (!std::isfinite(squared_limit) || omitted < config_.inlier_distance ||
      config_.z_hit * std::exp(-0.5 * (omitted / config_.sigma_hit) *
          (omitted / config_.sigma_hit)) + config_.z_random != config_.z_random)
    squared_limit = std::numeric_limits<float>::max();
  if (!queries.empty()) map_->squaredDistances(queries, squared_limit, distances, config_.num_threads);
  for (size_t i = 0; i < poses.size(); ++i) {
    GlobalHypothesis h;
    h.pose = poses[i];
    for (size_t j = 0; j < points_.size(); ++j) {
      const double d = distances[i * points_.size() + j];
      h.mean_squared_error += std::min(d, limit);
      h.inlier_ratio += d < limit;
      const double normalized_distance = std::sqrt(d) / config_.sigma_hit;
      h.score += (config_.z_hit * std::exp(-0.5 * normalized_distance * normalized_distance) +
          config_.z_random) /
          (config_.z_hit + config_.z_random);
    }
    h.score /= points_.size();
    h.inlier_ratio /= points_.size();
    h.mean_squared_error /= points_.size();
    progress_.best_inlier_ratio = std::max(progress_.best_inlier_ratio, h.inlier_ratio);
    retain(h);
    if (config_.refinement_levels > 0 &&
        progress_.refinement_level == config_.refinement_levels) local_samples_.push_back(h);
  }
  if (cursor_ == count) finishLevel();
  return progress_.complete;
}
void GlobalLocalizer::finishLevel() {
  std::stable_sort(best_.begin(), best_.end(), [](const auto &a, const auto &b) {
    return a.score > b.score;
  });
  if (!best_.empty() && progress_.refinement_level < config_.refinement_levels) {
    pending_.clear();
    spacing_ *= 0.5;
    for (const auto &h : best_)
      for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
          for (int a = -1; a <= 1; ++a) {
            Eigen::Vector3d v(h.pose.translation().x() + x * spacing_.x(),
                h.pose.translation().y() + y * spacing_.y(), yaw(h.pose, config_) + a * spacing_.z());
            // Rotate around the observed scene, not the base origin. Otherwise a yaw
            // correction can require a translation outside the shrinking search box.
            v.head<2>() += ((h.pose.rotation() - pose(v).rotation()) * scan_center_).head<2>();
            if (config_.yaw_max - config_.yaw_min >= 2 * pi - 1e-9)
              v.z() = config_.yaw_min + std::fmod(v.z() - config_.yaw_min + 2 * pi, 2 * pi);
            pending_.push_back(v);
          }
    ++progress_.refinement_level;
    progress_.total += pending_.size();
    cursor_ = 0;
    return;
  }
  result_.hypotheses = best_;
  progress_.complete = true;
  if (best_.empty()) { result_.reason = "no_valid_candidates"; return; }
  const auto &winner = best_.front();
  result_.pose = winner.pose;
  if (winner.inlier_ratio < config_.min_inlier_ratio ||
      winner.mean_squared_error > config_.max_mean_squared_error) {
    result_.reason = "insufficient_match"; return;
  }
  for (size_t i = 1; i < best_.size(); ++i)
    if (!sameMode(winner, best_[i], config_) &&
        winner.score - best_[i].score <= config_.ambiguity_score_margin) {
      result_.reason = "ambiguous_match"; return;
    }
  double weight = 0;
  for (const auto &h : local_samples_) {
    if (!sameMode(winner, h, config_)) continue;
    Eigen::Matrix<double, 6, 1> d = Eigen::Matrix<double, 6, 1>::Zero();
    d.head<2>() = (h.pose.translation() - winner.pose.translation()).head<2>();
    d(5) = angle(yaw(h.pose, config_) - yaw(winner.pose, config_));
    const double w = std::exp(20 * (h.score - winner.score));
    result_.covariance.noalias() += w * d * d.transpose();
    weight += w;
  }
  if (weight > 0) result_.covariance /= weight;
  result_.covariance(0, 0) += spacing_.x() * spacing_.x() / 12;
  result_.covariance(1, 1) += spacing_.y() * spacing_.y() / 12;
  result_.covariance(5, 5) += spacing_.z() * spacing_.z() / 12;
  if (result_.covariance(0, 0) + result_.covariance(1, 1) >
          config_.max_position_stddev * config_.max_position_stddev ||
      result_.covariance(5, 5) > config_.max_yaw_stddev * config_.max_yaw_stddev) {
    result_.reason = "diffuse_match"; return;
  }
  result_.success = true;
  result_.reason = "success";
}
const GlobalLocalizationResult &GlobalLocalizer::result() const {
  if (!progress_.complete) throw std::logic_error("global localization is incomplete");
  return result_;
}
}  // namespace emcl3d
