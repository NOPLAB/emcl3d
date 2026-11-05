// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#include <emcl3d/particle_filter.hpp>
#include <emcl3d/global_localizer.hpp>
#include <emcl3d/action/global_localization.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace emcl3d
{
namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

Pose pose_from_message(const geometry_msgs::msg::Pose & message)
{
  const Eigen::Vector3d translation(message.position.x, message.position.y, message.position.z);
  Eigen::Quaterniond rotation(
    message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z);
  require(translation.allFinite() && rotation.coeffs().allFinite() &&
    std::isfinite(rotation.norm()) && rotation.norm() > 1e-12,
    "pose contains an invalid translation or quaternion");
  rotation.normalize();
  Pose pose = Pose::Identity();
  pose.linear() = rotation.toRotationMatrix();
  pose.translation() = translation;
  return pose;
}

Pose pose_from_transform(const geometry_msgs::msg::TransformStamped & message)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = message.transform.translation.x;
  pose.position.y = message.transform.translation.y;
  pose.position.z = message.transform.translation.z;
  pose.orientation = message.transform.rotation;
  return pose_from_message(pose);
}

geometry_msgs::msg::Pose pose_message(const Pose & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.translation().x();
  message.position.y = pose.translation().y();
  message.position.z = pose.translation().z();
  Eigen::Quaterniond rotation(pose.linear());
  rotation.normalize();
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  message.orientation.w = rotation.w();
  return message;
}

Matrix6 checked_covariance(Matrix6 covariance)
{
  require(covariance.allFinite(), "covariance must be finite");
  require(covariance.isApprox(covariance.transpose(), 1e-9), "covariance must be symmetric");
  covariance = (0.5 * (covariance + covariance.transpose())).eval();
  const Eigen::SelfAdjointEigenSolver<Matrix6> solver(covariance);
  require(solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -1e-12,
    "covariance must be positive semidefinite");
  // Remove only roundoff-sized negative eigenvalues, not invalid covariance.
  return solver.eigenvectors() * solver.eigenvalues().cwiseMax(0.0).asDiagonal() *
         solver.eigenvectors().transpose();
}
}  // namespace

class Emcl3dNode : public rclcpp::Node
{
public:
  using GlobalAction = emcl3d::action::GlobalLocalization;
  using GlobalGoal = rclcpp_action::ServerGoalHandle<GlobalAction>;
  explicit Emcl3dNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("emcl3d", options), last_clock_(0, 0, get_clock()->get_clock_type()),
    last_stamp_(0, 0, get_clock()->get_clock_type()),
    initial_stamp_(0, 0, get_clock()->get_clock_type())
  {
    config_.num_particles = integer("num_particles", 500, 2, 1000000);
    config_.num_threads = integer(
      "num_threads", 0, std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
    config_.random_seed = static_cast<unsigned>(integer(
      "random_seed", 1, 0, std::numeric_limits<int>::max()));
    config_.resample_ess_ratio = number("resample_ess_ratio", 0.5, 0.0, 1.0, true);
    config_.sigma_hit = number("sigma_hit", 0.3, 0.0);
    config_.z_hit = number("z_hit", 0.95, 0.0, 1.0);
    config_.z_random = number("z_random", 0.05, 0.0, 1.0);
    require(config_.z_hit + config_.z_random > 0.0, "likelihood mixture cannot be zero");
    config_.likelihood_temperature = number("likelihood_temperature", 10.0, 0.0);
    config_.inlier_distance = number("inlier_distance", 0.5, 0.0);
    config_.min_inlier_ratio = number("min_inlier_ratio", 0.3, 0.0, 1.0);
    config_.alpha_threshold = number("alpha_threshold", 0.5, 0.0, 1.0, true);
    config_.reset_confirmation_updates = integer("reset_confirmation_updates", 3, 1, 1000000);
    config_.min_valid_points = integer("min_valid_points", 20, 1, 1000000);
    config_.translation_noise = vector_parameter("translation_noise", {0.1, 0.1, 0.1});
    config_.rotation_noise = vector_parameter("rotation_noise", {0.1, 0.1, 0.1});
    config_.expansion_translation = vector_parameter("expansion_translation", {0.2, 0.2, 0.2});
    config_.expansion_rotation = vector_parameter("expansion_rotation", {0.1, 0.1, 0.1});

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    for (const auto & frame : {map_frame_, odom_frame_, base_frame_}) {
      require(!frame.empty() && frame.front() != '/', "frame names must be nonempty without '/' prefix");
    }
    require(map_frame_ != odom_frame_ && map_frame_ != base_frame_ && odom_frame_ != base_frame_,
      "map, odom and base frames must be distinct");
    scan_voxel_ = number("scan_voxel_size", 0.2, 0.0);
    min_range_ = number("min_range", 0.0, 0.0, std::numeric_limits<double>::max(), true);
    max_range_ = number("max_range", 100.0, min_range_);
    max_age_ = number("max_cloud_age", 1.0, 0.0);
    future_tolerance_ = number("future_tolerance", 0.1, 0.0,
      std::numeric_limits<double>::max(), true);
    tf_wait_ = number("tf_wait_timeout", 0.3, 0.0);
    max_position_stddev_ = number("max_position_stddev", 2.0, 0.0);
    max_rotation_stddev_ = number("max_rotation_stddev", 0.5, 0.0);
    max_eval_points_ = integer("max_eval_points", 2000, config_.min_valid_points, 1000000);
    max_input_points_ = integer("max_input_points", 2000000, max_eval_points_, 100000000);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    const auto initial_pose = declare_parameter<std::vector<double>>(
      "initial_pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    const auto initial_variance = declare_parameter<std::vector<double>>(
      "initial_variance", {1.0, 1.0, 1.0, 0.1, 0.1, 0.5});
    require(initial_pose.size() == 6 && initial_variance.size() == 6,
      "initial_pose and initial_variance must contain six values");
    estimate_pose_ = Pose::Identity();
    estimate_covariance_ = Matrix6::Zero();
    for (int i = 0; i < 6; ++i) {
      require(std::isfinite(initial_pose[i]) && std::isfinite(initial_variance[i]) &&
        initial_variance[i] >= 0.0, "initial pose/variance must be finite; variance nonnegative");
      estimate_covariance_(i, i) = initial_variance[i];
    }
    estimate_pose_.translation() = Eigen::Vector3d(initial_pose[0], initial_pose[1], initial_pose[2]);
    estimate_pose_.linear() = (
      Eigen::AngleAxisd(initial_pose[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(initial_pose[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(initial_pose[3], Eigen::Vector3d::UnitX())).toRotationMatrix();

    const auto map_path = declare_parameter<std::string>("map_path", "");
    require(!map_path.empty(), "map_path must name a PCD map in map_frame");
    auto map = std::make_shared<Map>(declare_parameter<std::string>("compute_backend", "cpu"));
    map->load(map_path, number("map_voxel_size", 0.2, 0.0));
    map_ = map;
    filter_ = std::make_unique<ParticleFilter>(config_, map_);
    filter_->initialize(estimate_pose_, estimate_covariance_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // The TF listener only mutates the thread-safe TF buffer. All filter state is
    // confined to this node's default mutually-exclusive callback group.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_pose", 1);
    particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 1);
    alpha_pub_ = create_publisher<std_msgs::msg::Float64>("alpha", 1);
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("cloud_topic", "points"),
      rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { receive_cloud(std::move(cloud)); });
    initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(1),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose) {
        receive_initial_pose(*pose);
      });
    setup_global_localization();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      process_cloud();
      process_global_localization();
    });
    diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      observe_clock();
      if (last_stamp_.nanoseconds() > 0 && (now() - last_stamp_).seconds() > max_age_) {
        status_level_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        status_text_ = "No recent accepted observation";
      }
      publish_diagnostics();
    });
    last_clock_ = now();
  }

private:
  struct Observation {
    std::vector<Eigen::Vector3d> points;
    Pose odom;
    rclcpp::Time stamp;
  };

  static std::vector<Eigen::Vector3d> sampled_points(
    const std::vector<Eigen::Vector3d> & input, size_t limit)
  {
    if (input.size() <= limit) {
      return input;
    }
    std::vector<Eigen::Vector3d> output;
    output.reserve(limit);
    for (size_t i = 0; i < limit; ++i) {
      output.push_back(input[i * input.size() / limit]);
    }
    return output;
  }

  bool global_busy() const
  {
    return global_goal_ || global_search_ || global_waiting_;
  }

  void setup_global_localization()
  {
    const std::string prefix = "global_localization.";
    global_config_.candidates = integer(prefix + "num_candidates", 100000, 2, 10000000);
    global_config_.batch_size = integer(prefix + "batch_size", 256, 1, 1000000);
    global_eval_points_ = integer(prefix + "max_eval_points", 2000, config_.min_valid_points, 1000000);
    require(global_config_.batch_size * static_cast<size_t>(global_eval_points_) <= 16000000,
      "global localization batch_size * max_eval_points must not exceed 16000000");
    global_config_.max_eval_points = global_eval_points_;
    global_config_.min_valid_points = config_.min_valid_points;
    global_config_.num_hypotheses = integer(prefix + "num_hypotheses", 10, 2, 1000);
    global_config_.refinement_levels = integer(prefix + "refinement_levels", 3, 0, 10);
    global_config_.free_cells_only = declare_parameter<bool>(prefix + "free_cells_only", true);
    global_config_.footprint_radius = number(prefix + "footprint_radius", 0.3, 0.0,
      std::numeric_limits<double>::max(), true);
    global_config_.inlier_distance = config_.inlier_distance;
    global_config_.sigma_hit = config_.sigma_hit;
    global_config_.z_hit = config_.z_hit;
    global_config_.z_random = config_.z_random;
    global_config_.num_threads = config_.num_threads;
    global_config_.min_inlier_ratio = number(prefix + "min_inlier_ratio", 0.6, 0.0, 1.0);
    global_config_.max_mean_squared_error = number(prefix + "max_mean_squared_error", 0.15, 0.0);
    global_config_.ambiguity_score_margin = number(prefix + "ambiguity_score_margin", 0.02, 0.0);
    global_config_.mode_separation_xy = number(prefix + "mode_separation_xy", 1.0, 0.0);
    global_config_.mode_separation_yaw = number(prefix + "mode_separation_yaw", 0.5, 0.0);
    global_config_.max_position_stddev = number(prefix + "max_position_stddev", 0.5, 0.0);
    global_config_.max_yaw_stddev = number(prefix + "max_yaw_stddev", 0.2, 0.0);
    global_config_.seed = config_.random_seed;
    require(declare_parameter<std::string>(prefix + "search_mode", "xy_yaw") == "xy_yaw",
      "global localization currently supports only xy_yaw search");
    const auto fixed = declare_parameter<std::vector<double>>(prefix + "fixed_z_roll_pitch",
      {0.0, 0.0, 0.0});
    require(fixed.size() == 3 && std::all_of(fixed.begin(), fixed.end(),
      [](double value) {return std::isfinite(value);}),
      "fixed_z_roll_pitch must contain three finite values");
    global_config_.z = fixed[0];
    global_config_.roll = fixed[1];
    global_config_.pitch = fixed[2];
    const auto yaw = declare_parameter<std::vector<double>>(prefix + "yaw_range",
      {-3.141592653589793, 3.141592653589793});
    require(yaw.size() == 2 && std::isfinite(yaw[0]) && std::isfinite(yaw[1]) &&
      yaw[1] > yaw[0] && yaw[1] - yaw[0] <= 2 * 3.141592653589793 + 1e-12,
      "yaw_range must be [min, max] spanning at most 2*pi");
    global_config_.yaw_min = yaw[0];
    global_config_.yaw_max = yaw[1];
    const auto bounds = declare_parameter<std::vector<double>>(
      prefix + "search_bounds", {0.0, 0.0, 0.0, 0.0});
    require(bounds.size() == 4, "search_bounds must contain [xmin,xmax,ymin,ymax]; all zeros uses map bounds");
    if (!std::all_of(bounds.begin(), bounds.end(), [](double value) {return value == 0.0;})) {
      set_search_bounds(global_config_, {bounds[0], bounds[1], bounds[2], bounds[3]});
    }
    global_timeout_ = number(prefix + "timeout", 30.0, 0.0);
    global_wait_timeout_ = number(prefix + "startup_wait_timeout", 30.0, 0.0);
    global_apply_ = declare_parameter<bool>(prefix + "apply_result", true);
    startup_waiting_ = declare_parameter<bool>(prefix + "run_on_startup", false);
    require(!startup_waiting_ || global_apply_,
      "run_on_startup requires global_localization.apply_result=true");
    startup_unlocalized_ = startup_waiting_;
    global_wait_started_ = std::chrono::steady_clock::now();
    occupancy_threshold_ = integer(prefix + "occupancy_threshold", 50, 1, 100);
    free_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter<std::string>(prefix + "free_space_topic", "map"),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        try {
          require(message->header.frame_id == map_frame_, "free-space grid must use map_frame");
          FreeSpaceGrid grid;
          grid.width = message->info.width;
          grid.height = message->info.height;
          grid.resolution = message->info.resolution;
          grid.origin = pose_from_message(message->info.origin);
          grid.data = message->data;
          grid.occupancy_threshold = occupancy_threshold_;
          grid.validate();
          free_grid_ = std::move(grid);
          ++free_grid_updates_;
          publish_diagnostics();
        } catch (const std::exception & error) {
          free_grid_.reset();
          status(diagnostic_msgs::msg::DiagnosticStatus::WARN,
            std::string("Rejected free-space grid: ") + error.what());
        }
      });
    global_server_ = rclcpp_action::create_server<GlobalAction>(
      this, "~/global_localization",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GlobalAction::Goal> goal) {
        if (global_busy() || startup_waiting_) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        try {
          auto config = global_config_;
          if (goal->use_search_bounds) {
            set_search_bounds(config, goal->search_bounds);
          }
        } catch (const std::invalid_argument &) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GlobalGoal>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<GlobalGoal> goal) {
        global_goal_ = goal;
        global_waiting_ = true;
        global_wait_started_ = std::chrono::steady_clock::now();
        global_state_ = "waiting_for_input";
        global_code_.clear();
        global_evaluated_ = 0;
      });
    if (startup_waiting_) {
      global_state_ = "waiting_for_input";
      status(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for startup global localization");
    }
  }

  static void set_search_bounds(GlobalLocalizationConfig & config, const std::array<double, 4> & bounds)
  {
    require(std::all_of(bounds.begin(), bounds.end(), [](double value) {return std::isfinite(value);}) &&
      bounds[0] < bounds[1] && bounds[2] < bounds[3],
      "search_bounds requires finite increasing XY intervals");
    config.min_xy = Eigen::Vector2d(bounds[0], bounds[2]);
    config.max_xy = Eigen::Vector2d(bounds[1], bounds[3]);
  }

  void stop_global_localization(const std::string & code, const std::string & message,
    bool canceled = false)
  {
    if (!global_busy() && !startup_waiting_) {
      return;
    }
    global_code_ = code;
    global_state_ = canceled ? "canceled" : "failed";
    if (global_goal_) {
      auto result = std::make_shared<GlobalAction::Result>();
      result->code = code;
      result->message = message;
      if (canceled || global_goal_->is_canceling()) {
        global_goal_->canceled(result);
      } else {
        global_goal_->abort(result);
      }
    }
    global_goal_.reset();
    global_search_.reset();
    search_observation_.reset();
    global_waiting_ = false;
    startup_waiting_ = false;
    status(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    publish_diagnostics();
  }

  void process_global_localization()
  {
    if (global_goal_ && global_goal_->is_canceling()) {
      stop_global_localization("CANCELED", "Global localization canceled", true);
      return;
    }
    if (!global_busy() && !startup_waiting_) {
      return;
    }
    const auto current = std::chrono::steady_clock::now();
    if (!global_search_) {
      const bool recent = latest_observation_ &&
        (now() - latest_observation_->stamp).seconds() <= max_age_ &&
        (now() - latest_observation_->stamp).seconds() >= -future_tolerance_;
      if (!recent || (global_config_.free_cells_only && !free_grid_)) {
        if (std::chrono::duration<double>(current - global_wait_started_).count() >= global_wait_timeout_) {
          stop_global_localization(!recent ? "NO_CLOUD" : "NO_FREE_SPACE",
            !recent ? "No valid cloud and observation-stamped TF before input timeout" :
            "No valid free-space grid before input timeout");
        } else {
          publish_global_feedback("waiting_for_input", 0, 0, 0);
        }
        return;
      }
      try {
        auto config = global_config_;
        if (global_goal_ && global_goal_->get_goal()->use_search_bounds) {
          set_search_bounds(config, global_goal_->get_goal()->search_bounds);
        }
        search_observation_ = latest_observation_;
        global_search_ = std::make_unique<GlobalLocalizer>(map_, config,
          sampled_points(search_observation_->points, global_eval_points_), free_grid_);
        global_search_started_ = current;
        global_waiting_ = false;
        startup_waiting_ = false;
        global_state_ = "searching";
        global_evaluated_ = 0;
      } catch (const std::exception & error) {
        stop_global_localization("INVALID_INPUT", error.what());
        return;
      }
    }
    if (std::chrono::duration<double>(current - global_search_started_).count() >= global_timeout_) {
      stop_global_localization("TIMEOUT", "Global localization search timed out");
      return;
    }
    try {
      global_search_->step();
      if (std::chrono::duration<double>(
          std::chrono::steady_clock::now() - global_search_started_).count() >= global_timeout_)
      {
        stop_global_localization("TIMEOUT", "Global localization search timed out");
        return;
      }
      const auto & progress = global_search_->progress();
      global_evaluated_ = progress.evaluated;
      publish_global_feedback(progress.refinement_level == 0 ? "searching" : "refining",
        progress.evaluated, progress.total, progress.best_inlier_ratio);
      if (progress.complete) {
        finish_global_localization();
      }
    } catch (const std::exception & error) {
      stop_global_localization("SEARCH_ERROR", error.what());
    }
  }

  void publish_global_feedback(const std::string & stage, size_t evaluated, size_t total, double best)
  {
    if (!global_goal_) {
      return;
    }
    auto feedback = std::make_shared<GlobalAction::Feedback>();
    feedback->stage = stage;
    feedback->evaluated_candidates = evaluated;
    feedback->total_candidates = total;
    feedback->best_inlier_ratio = best;
    feedback->elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - global_wait_started_).count();
    global_goal_->publish_feedback(feedback);
  }

  void finish_global_localization()
  {
    const auto & found = global_search_->result();
    auto result = std::make_shared<GlobalAction::Result>();
    result->success = found.success;
    result->code = found.reason;
    result->message = found.reason;
    result->pose.header.frame_id = map_frame_;
    result->pose.header.stamp = search_observation_->stamp;
    result->pose.pose.pose = pose_message(found.pose);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        result->pose.pose.covariance[6 * row + col] = found.covariance(row, col);
      }
    }
    for (const auto & hypothesis : found.hypotheses) {
      result->hypotheses.push_back(pose_message(hypothesis.pose));
      result->scores.push_back(hypothesis.score);
      result->inlier_ratios.push_back(hypothesis.inlier_ratio);
    }
    const bool apply = global_apply_ && (!global_goal_ || !global_goal_->get_goal()->estimate_only);
    if (found.success && apply) {
      if (!latest_observation_ || (now() - latest_observation_->stamp).seconds() > max_age_) {
        result->success = false;
        result->code = "STALE_OBSERVATION";
        result->message = "Search succeeded but no recent observation is available for handoff";
      } else {
        const auto & latest = *latest_observation_;
        const Pose delta = search_observation_->odom.inverse() * latest.odom;
        const Pose pose = found.pose * delta;
        // Fixed-map-axis orientation uncertainty moves the translated base origin.
        const Eigen::Vector3d arm = found.pose.rotation() * delta.translation();
        Eigen::Matrix3d skew;
        skew << 0, -arm.z(), arm.y(), arm.z(), 0, -arm.x(), -arm.y(), arm.x(), 0;
        Matrix6 jacobian = Matrix6::Identity();
        jacobian.topRightCorner<3, 3>() = -skew;
        Matrix6 covariance = jacobian * found.covariance * jacobian.transpose();
        const double motion = delta.translation().norm() +
          std::abs(Eigen::AngleAxisd(delta.rotation()).angle());
        covariance.topLeftCorner<3, 3>() += pose.rotation() *
          (config_.translation_noise.array().square() * motion).matrix().asDiagonal() *
          pose.rotation().transpose();
        covariance.bottomRightCorner<3, 3>() += pose.rotation() *
          (config_.rotation_noise.array().square() * motion).matrix().asDiagonal() *
          pose.rotation().transpose();
        filter_->initialize(pose, covariance);
        estimate_pose_ = pose;
        estimate_covariance_ = covariance;
        last_odom_ = latest.odom;
        last_stamp_ = latest.stamp;
        initial_stamp_ = latest.stamp;
        have_odom_ = true;
        filter_usable_ = true;
        startup_unlocalized_ = false;
        alpha_ = 0;
        support_ratio_ = found.hypotheses.empty() ? 0 : found.hypotheses.front().inlier_ratio;
        last_reset_ = false;
        if (pending_ && rclcpp::Time(pending_->header.stamp, get_clock()->get_clock_type()) <= latest.stamp) {
          pending_.reset();
        }
        publish_estimate(latest.stamp, latest.odom);
        result->applied = true;
      }
    }
    global_code_ = result->code;
    global_state_ = result->success ? "succeeded" : "failed";
    if (global_goal_) {
      if (result->success) {
        global_goal_->succeed(result);
      } else {
        global_goal_->abort(result);
      }
    }
    global_goal_.reset();
    global_search_.reset();
    search_observation_.reset();
    global_waiting_ = false;
    status(result->success ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      std::string("Global localization: ") + result->code +
      (result->applied ? " (applied)" : " (not applied)"));
    publish_diagnostics();
  }

  int integer(const std::string & name, int value, int minimum, int maximum)
  {
    const auto result = declare_parameter<int64_t>(name, value);
    require(result >= minimum && result <= maximum, name + " is outside its allowed range");
    return static_cast<int>(result);
  }

  double number(
    const std::string & name, double value, double minimum,
    double maximum = std::numeric_limits<double>::max(), bool include_minimum = false)
  {
    const double result = declare_parameter<double>(name, value);
    require(std::isfinite(result) && result <= maximum &&
      (include_minimum ? result >= minimum : result > minimum),
      name + " must be finite and in its allowed range");
    return result;
  }

  Eigen::Vector3d vector_parameter(const std::string & name, const std::vector<double> & value)
  {
    const auto result = declare_parameter<std::vector<double>>(name, value);
    require(result.size() == 3, name + " must contain three standard deviations");
    Eigen::Vector3d vector;
    for (int i = 0; i < 3; ++i) {
      require(std::isfinite(result[i]) && result[i] >= 0.0, name + " must be finite and nonnegative");
      vector[i] = result[i];
    }
    return vector;
  }

  void status(uint8_t level, const std::string & message)
  {
    status_level_ = level;
    status_text_ = message;
  }

  void observe_clock()
  {
    const auto current = now();
    if (current < last_clock_) {
      stop_global_localization("CLOCK_RESET", "ROS time moved backwards");
      latest_observation_.reset();
      pending_.reset();
      last_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      initial_stamp_ = last_stamp_;
      have_odom_ = false;
      // Retain the last map belief, but never integrate across clock epochs.
      filter_->initialize(estimate_pose_, estimate_covariance_);
      filter_usable_ = true;
      status(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "ROS time moved backwards; odometry baseline reset");
    }
    last_clock_ = current;
  }

  void receive_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    observe_clock();
    const rclcpp::Time stamp(cloud->header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || cloud->header.frame_id.empty() ||
      stamp <= last_stamp_ || stamp < initial_stamp_ || age > max_age_ ||
      age < -future_tolerance_)
    {
      ++dropped_;
      status(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Dropped zero, stale, future, out-of-order or pre-initialization cloud");
      return;
    }
    if (pending_ && stamp <= rclcpp::Time(pending_->header.stamp, get_clock()->get_clock_type())) {
      ++dropped_;
      return;
    }
    if (pending_) {
      ++dropped_;
    }
    pending_ = std::move(cloud);
    pending_since_ = std::chrono::steady_clock::now();
  }

  void receive_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & message)
  {
    observe_clock();
    try {
      require(!message.header.frame_id.empty(), "initialpose requires a frame_id");
      auto stamp = rclcpp::Time(message.header.stamp, get_clock()->get_clock_type());
      if (stamp.nanoseconds() == 0) {
        stamp = now();
      }
      require(stamp.nanoseconds() > 0 && (stamp - now()).seconds() <= future_tolerance_,
        "initialpose requires a valid, non-future stamp");
      Pose pose = pose_from_message(message.pose.pose);
      Matrix6 covariance;
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          covariance(row, col) = message.pose.covariance[6 * row + col];
        }
      }
      covariance = checked_covariance(covariance);
      if (message.header.frame_id != map_frame_) {
        const auto transform = pose_from_transform(tf_buffer_->lookupTransform(
          map_frame_, message.header.frame_id, stamp, rclcpp::Duration::from_seconds(0.0)));
        pose = transform * pose;
        Matrix6 rotation = Matrix6::Zero();
        rotation.topLeftCorner<3, 3>() = transform.linear();
        rotation.bottomRightCorner<3, 3>() = transform.linear();
        covariance = rotation * covariance * rotation.transpose();
      }
      const auto odom = pose_from_transform(tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.0)));
      stop_global_localization("MANUAL_INITIALPOSE", "Manual initial pose takes priority");
      startup_waiting_ = false;
      startup_unlocalized_ = false;
      latest_observation_.reset();
      filter_->initialize(pose, covariance);
      estimate_pose_ = pose;
      estimate_covariance_ = covariance;
      last_odom_ = odom;
      have_odom_ = true;
      filter_usable_ = true;
      initial_stamp_ = stamp;
      last_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      pending_.reset();
      alpha_ = 0.0;
      support_ratio_ = 0.0;
      status(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Initial pose accepted; awaiting supported observation");
      publish_estimate(stamp, odom);
      publish_diagnostics();
    } catch (const std::exception & error) {
      status(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        std::string("Rejected initialpose: ") + error.what());
      RCLCPP_WARN(get_logger(), "%s", status_text_.c_str());
    }
  }

  static double coordinate(
    const uint8_t * data, const sensor_msgs::msg::PointField & field, bool swap)
  {
    std::array<uint8_t, 8> bytes{};
    const size_t size = field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
    std::memcpy(bytes.data(), data + field.offset, size);
    if (swap) {
      std::reverse(bytes.begin(), bytes.begin() + size);
    }
    if (size == 4) {
      float value;
      std::memcpy(&value, bytes.data(), 4);
      return value;
    }
    double value;
    std::memcpy(&value, bytes.data(), 8);
    return value;
  }

  std::vector<Eigen::Vector3d> points(
    const sensor_msgs::msg::PointCloud2 & cloud, const Pose & base_from_sensor,
    int limit = 0)
  {
    if (limit == 0) {
      limit = max_eval_points_;
    }
    const uint64_t count = static_cast<uint64_t>(cloud.width) * cloud.height;
    require(count > 0 && count <= static_cast<uint64_t>(max_input_points_),
      "cloud point count is empty or exceeds max_input_points");
    require(cloud.point_step > 0 && static_cast<uint64_t>(cloud.row_step) >=
      static_cast<uint64_t>(cloud.width) * cloud.point_step &&
      static_cast<uint64_t>(cloud.row_step) * cloud.height <= cloud.data.size(),
      "malformed PointCloud2 storage");
    std::array<sensor_msgs::msg::PointField, 3> fields;
    for (size_t axis = 0; axis < 3; ++axis) {
      const std::string name(1, "xyz"[axis]);
      const auto found = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&name](const auto & field) {return field.name == name;});
      require(found != cloud.fields.end(), "cloud requires x, y and z fields");
      require(found->count == 1 &&
        (found->datatype == sensor_msgs::msg::PointField::FLOAT32 ||
        found->datatype == sensor_msgs::msg::PointField::FLOAT64),
        "xyz fields must be scalar FLOAT32 or FLOAT64");
      const uint64_t size = found->datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
      require(static_cast<uint64_t>(found->offset) + size <= cloud.point_step,
        "xyz field extends beyond point_step");
      fields[axis] = *found;
    }
    const uint16_t endian_probe = 1;
    const bool little_endian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 1;
    const bool swap = cloud.is_bigendian == little_endian;
    std::set<std::array<int64_t, 3>> voxels;
    std::vector<Eigen::Vector3d> result;
    result.reserve(static_cast<size_t>(limit));
    // Uniform index sampling bounds work and avoids taking only the scan's first sector.
    const uint64_t stride = std::max<uint64_t>(
      1, (count + static_cast<uint64_t>(limit) - 1) / limit);
    for (uint64_t index = 0; index < count; index += stride) {
      const auto * data = cloud.data.data() + (index / cloud.width) * cloud.row_step +
        (index % cloud.width) * cloud.point_step;
      Eigen::Vector3d point(
        coordinate(data, fields[0], swap), coordinate(data, fields[1], swap),
        coordinate(data, fields[2], swap));
      if (!point.allFinite()) {
        continue;
      }
      const double range = point.norm();
      if (!std::isfinite(range) || range < min_range_ || range > max_range_) {
        continue;
      }
      point = base_from_sensor * point;
      if (!point.allFinite()) {
        continue;
      }
      std::array<int64_t, 3> key{};
      bool valid = true;
      for (int axis = 0; axis < 3; ++axis) {
        const double voxel = std::floor(point[axis] / scan_voxel_);
        // Conservative bound also excludes float-to-integer overflow.
        if (!std::isfinite(voxel) || std::abs(voxel) >= 9e18) {
          valid = false;
          break;
        }
        key[axis] = static_cast<int64_t>(voxel);
      }
      if (valid && voxels.insert(key).second) {
        result.push_back(point);
      }
    }
    require(result.size() >= static_cast<size_t>(config_.min_valid_points),
      "too few finite, distinct voxel points after preprocessing");
    return result;
  }

  void process_cloud()
  {
    observe_clock();
    if (!pending_ || (!filter_usable_ && !startup_unlocalized_ && !global_busy())) {
      return;
    }
    const auto processing_started = std::chrono::steady_clock::now();
    const rclcpp::Time stamp(pending_->header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (age > max_age_ || age < -future_tolerance_) {
      pending_.reset();
      ++dropped_;
      status(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Pending cloud expired");
      return;
    }
    Pose odom;
    Pose base_from_sensor = Pose::Identity();
    try {
      odom = pose_from_transform(tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.0)));
      if (pending_->header.frame_id != base_frame_) {
        base_from_sensor = pose_from_transform(tf_buffer_->lookupTransform(
          base_frame_, pending_->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.0)));
      }
    } catch (const tf2::TransformException & error) {
      status(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        std::string("Waiting for observation-stamped TF: ") + error.what());
      if (std::chrono::duration<double>(
          std::chrono::steady_clock::now() - pending_since_).count() >= tf_wait_)
      {
        pending_.reset();
        ++dropped_;
      }
      return;
    } catch (const std::exception & error) {
      pending_.reset();
      ++dropped_;
      status(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        std::string("Invalid observation transform: ") + error.what());
      return;
    }
    std::vector<Eigen::Vector3d> observation;
    std::vector<Eigen::Vector3d> global_points;
    try {
      observation = points(*pending_, base_from_sensor,
        startup_unlocalized_ ? global_eval_points_ : max_eval_points_);
    } catch (const std::exception & error) {
      pending_.reset();
      ++dropped_;
      status(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        std::string("Rejected cloud: ") + error.what());
      return;
    }
    if (startup_unlocalized_ || global_eval_points_ == max_eval_points_) {
      global_points = observation;
    } else {
      try {
        global_points = points(*pending_, base_from_sensor, global_eval_points_);
      } catch (const std::invalid_argument &) {
        // A sparse global sample must not reject a valid tracking observation.
        global_points.clear();
      }
    }
    pending_.reset();
    if (global_points.empty()) {
      latest_observation_.reset();
    } else {
      latest_observation_ = Observation{std::move(global_points), odom, stamp};
    }
    if (startup_unlocalized_ || !filter_usable_) {
      return;
    }
    const Pose delta = have_odom_ ? Pose(last_odom_.inverse() * odom) : Pose::Identity();
    try {
      const auto estimate = filter_->update(delta, observation);
      require(estimate.pose.matrix().allFinite() && estimate.covariance.allFinite() &&
        std::isfinite(estimate.alpha) && std::isfinite(estimate.ess),
        "filter returned a nonfinite estimate");
      estimate_pose_ = estimate.pose;
      estimate_covariance_ = estimate.covariance;
      alpha_ = estimate.alpha;
      ess_ = estimate.ess;
      last_reset_ = estimate.reset;
      if (last_reset_) {
        ++expansion_reset_count_;
      }
    } catch (const std::exception & error) {
      // update may have mutated particles before throwing: never retry its delta.
      filter_usable_ = false;
      status(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        std::string("Filter update failed; send initialpose: ") + error.what());
      return;
    }
    last_odom_ = odom;
    have_odom_ = true;
    last_stamp_ = stamp;
    ++accepted_;
    evaluated_points_ = observation.size();
    size_t inliers = 0;
    for (const auto & point : observation) {
      const double distance = map_->distance(estimate_pose_ * point);
      if (std::isfinite(distance) && distance <= config_.inlier_distance) {
        ++inliers;
      }
    }
    support_ratio_ = static_cast<double>(inliers) / observation.size();
    const bool supported = inliers > 0 && support_ratio_ >= config_.min_inlier_ratio;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> position_solver(
      estimate_covariance_.topLeftCorner<3, 3>());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_solver(
      estimate_covariance_.bottomRightCorner<3, 3>());
    const bool covariance_bounded = position_solver.info() == Eigen::Success &&
      rotation_solver.info() == Eigen::Success &&
      position_solver.eigenvalues().minCoeff() >= -1e-12 &&
      rotation_solver.eigenvalues().minCoeff() >= -1e-12 &&
      position_solver.eigenvalues().maxCoeff() <= max_position_stddev_ * max_position_stddev_ &&
      rotation_solver.eigenvalues().maxCoeff() <= max_rotation_stddev_ * max_rotation_stddev_;
    const bool inside_map = map_->contains(estimate_pose_.translation(), config_.inlier_distance);
    const bool healthy = supported && covariance_bounded && inside_map &&
      alpha_ >= config_.alpha_threshold && !last_reset_;
    status(healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      !inside_map ? "Estimated base pose is outside map bounds" :
      (!covariance_bounded ? "Pose covariance is large or invalid; localization is not confirmed" :
      (!supported ? "Insufficient map support; localization is not confirmed" :
      (last_reset_ ? "Expansion reset applied; localization is not confirmed" :
      (healthy ? "Observation supported (not a convergence guarantee)" : "Low localization alpha")))));
    processing_duration_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_started).count();
    publish_estimate(stamp, odom);
    publish_diagnostics();
  }

  void publish_estimate(const rclcpp::Time & stamp, const Pose & odom)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = stamp;
    pose.pose.pose = pose_message(estimate_pose_);
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        pose.pose.covariance[6 * row + col] = estimate_covariance_(row, col);
      }
    }
    pose_pub_->publish(pose);
    geometry_msgs::msg::PoseArray particles;
    particles.header = pose.header;
    particles.poses.reserve(filter_->particles().size());
    for (const auto & particle : filter_->particles()) {
      particles.poses.push_back(pose_message(particle.pose));
    }
    particles_pub_->publish(particles);
    std_msgs::msg::Float64 alpha;
    alpha.data = alpha_;
    alpha_pub_->publish(alpha);
    if (publish_tf_) {
      const auto correction = pose_message(estimate_pose_ * odom.inverse());
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = odom_frame_;
      transform.transform.translation.x = correction.position.x;
      transform.transform.translation.y = correction.position.y;
      transform.transform.translation.z = correction.position.z;
      transform.transform.rotation = correction.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus entry;
    entry.name = std::string(get_fully_qualified_name()) + "/localization";
    entry.hardware_id = "emcl3d";
    entry.level = status_level_;
    entry.message = status_text_;
    const auto value = [&entry](const std::string & key, const std::string & text) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = text;
        entry.values.push_back(std::move(item));
      };
    value("alpha", std::to_string(alpha_));
    value("ess", std::to_string(ess_));
    value("support_ratio", std::to_string(support_ratio_));
    value("evaluated_points", std::to_string(evaluated_points_));
    value("accepted_clouds", std::to_string(accepted_));
    value("dropped_clouds", std::to_string(dropped_));
    value("pending_clouds", pending_ ? "1" : "0");
    value("last_observation_stamp_ns", std::to_string(last_stamp_.nanoseconds()));
    value("expansion_reset", last_reset_ ? "true" : "false");
    value("expansion_reset_count", std::to_string(expansion_reset_count_));
    value("processing_duration_ms", std::to_string(processing_duration_ms_));
    value("global_localization_state", global_state_);
    value("global_localization_code", global_code_);
    value("global_localization_evaluated", std::to_string(global_evaluated_));
    value("startup_localization_pending", startup_unlocalized_ ? "true" : "false");
    value("global_localization_grid_updates", std::to_string(free_grid_updates_));
    message.status.push_back(std::move(entry));
    diagnostic_pub_->publish(message);
  }

  Config config_;
  GlobalLocalizationConfig global_config_;
  std::optional<FreeSpaceGrid> free_grid_;
  std::optional<Observation> latest_observation_, search_observation_;
  std::unique_ptr<GlobalLocalizer> global_search_;
  rclcpp_action::Server<GlobalAction>::SharedPtr global_server_;
  std::shared_ptr<GlobalGoal> global_goal_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr free_grid_sub_;
  bool startup_waiting_ = false, startup_unlocalized_ = false, global_waiting_ = false;
  bool global_apply_ = true;
  int global_eval_points_ = 2000, occupancy_threshold_ = 50;
  double global_timeout_ = 30, global_wait_timeout_ = 30;
  size_t global_evaluated_ = 0, free_grid_updates_ = 0;
  std::string global_state_ = "idle", global_code_;
  std::chrono::steady_clock::time_point global_wait_started_, global_search_started_;
  std::shared_ptr<const Map> map_;
  std::unique_ptr<ParticleFilter> filter_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alpha_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostic_timer_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_;
  std::chrono::steady_clock::time_point pending_since_;
  std::string map_frame_, odom_frame_, base_frame_;
  Pose estimate_pose_ = Pose::Identity(), last_odom_ = Pose::Identity();
  Matrix6 estimate_covariance_ = Matrix6::Identity();
  rclcpp::Time last_clock_, last_stamp_, initial_stamp_;
  bool have_odom_ = false, filter_usable_ = true, publish_tf_ = true, last_reset_ = false;
  double scan_voxel_, max_age_, future_tolerance_, tf_wait_;
  double min_range_, max_range_;
  double max_position_stddev_, max_rotation_stddev_;
  double alpha_ = 0.0, ess_ = 0.0, support_ratio_ = 0.0, processing_duration_ms_ = 0.0;
  int max_eval_points_, max_input_points_;
  uint64_t dropped_ = 0, accepted_ = 0, expansion_reset_count_ = 0;
  size_t evaluated_points_ = 0;
  uint8_t status_level_ = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  std::string status_text_ = "Awaiting first supported observation";
};
}  // namespace emcl3d

#ifndef EMCL3D_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<emcl3d::Emcl3dNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("emcl3d"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
