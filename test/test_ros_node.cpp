// SPDX-FileCopyrightText: 2026 nop <noplab90@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-only
#define EMCL3D_NO_MAIN
#include "../src/emcl3d_node.cpp"
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <future>

namespace {
class RosNodeTest : public testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  rclcpp::NodeOptions node_options() {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/emcl3d_test"});
    options.parameter_overrides({
      {"map_path", std::string(TEST_MAP_PATH)}, {"num_particles", 20}, {"num_threads", 1},
      {"min_valid_points", 4}, {"scan_voxel_size", 0.01},
      {"map_voxel_size", 0.01}, {"max_cloud_age", 30.0},
      {"tf_wait_timeout", 5.0},
      {"future_tolerance", 10.0},
      {"initial_variance", std::vector<double>(6, 0.0)},
      {"translation_noise", std::vector<double>(3, 0.0)},
      {"rotation_noise", std::vector<double>(3, 0.0)}});
    return options;
  }
  void SetUp() override {
    auto options = node_options();
    localization = std::make_shared<emcl3d::Emcl3dNode>(options);
    driver = std::make_shared<rclcpp::Node>("driver", "/emcl3d_test");
    cloud_pub = driver->create_publisher<sensor_msgs::msg::PointCloud2>(
      "points", rclcpp::SensorDataQoS());
    executor.add_node(localization);
    executor.add_node(driver);
    const auto graph = driver->get_graph_event();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (cloud_pub->get_subscription_count() == 0) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline);
      driver->wait_for_graph_change(graph, deadline - std::chrono::steady_clock::now());
      graph->check_and_clear();
    }
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(driver);
    geometry_msgs::msg::TransformStamped odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.header.stamp = driver->now();
    odom.transform.rotation.w = 1;
    odom.transform.translation.z = 0.7;
    broadcaster->sendTransform(odom);
  }
  void TearDown() override {
    executor.remove_node(localization);
    executor.remove_node(driver);
    localization.reset();
    driver.reset();
  }
  sensor_msgs::msg::PointCloud2 cloud() {
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = "base_link";
    message.header.stamp = driver->now();
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(8);
    sensor_msgs::PointCloud2Iterator<float> x(message,"x"), y(message,"y"), z(message,"z");
    for (int i=0; i<8; ++i, ++x, ++y, ++z) {
      *x = (i & 1) ? 2 : 0;
      *y = (i & 2) ? 3 : 0;
      *z = (i & 4) ? 4 : 0;
    }
    return message;
  }
  template<typename Future> void await(Future &future) {
    if (executor.spin_until_future_complete(future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Timed out awaiting ROS event");
    }
  }
  void configure_global(std::vector<rclcpp::Parameter> extra = {}) {
    executor.remove_node(localization);
    localization.reset();
    auto options = node_options();
    auto parameters = options.parameter_overrides();
    parameters.insert(parameters.end(), {
      {"global_localization.free_cells_only", false},
      {"global_localization.num_candidates", 128},
      {"global_localization.batch_size", 32},
      {"global_localization.refinement_levels", 1},
      {"global_localization.search_bounds", std::vector<double>{0.0, 0.02, 0.0, 0.02}},
      {"global_localization.yaw_range", std::vector<double>{0.0, 0.01}}});
    for (const auto & parameter : extra) {
      parameters.erase(std::remove_if(parameters.begin(), parameters.end(), [&](const auto & existing) {
        return existing.get_name() == parameter.get_name();
      }), parameters.end());
      parameters.push_back(parameter);
    }
    options.parameter_overrides(parameters);
    localization = std::make_shared<emcl3d::Emcl3dNode>(options);
    executor.add_node(localization);
    auto graph = localization->get_graph_event();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (localization->count_publishers("points") == 0) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline);
      localization->wait_for_graph_change(graph, deadline - std::chrono::steady_clock::now());
      graph->check_and_clear();
    }
    discover_publisher(cloud_pub);
  }
  template<typename T> void discover_publisher(const std::shared_ptr<T> & publisher) {
    auto graph = driver->get_graph_event();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (publisher->get_subscription_count() == 0) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline);
      driver->wait_for_graph_change(graph, deadline - std::chrono::steady_clock::now());
      graph->check_and_clear();
    }
  }
  template<typename T> void discover(const std::shared_ptr<T> &subscription) {
    auto graph = driver->get_graph_event();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (subscription->get_publisher_count() == 0) {
      ASSERT_LT(std::chrono::steady_clock::now(), deadline);
      driver->wait_for_graph_change(graph, deadline - std::chrono::steady_clock::now());
      graph->check_and_clear();
    }
  }
  std::shared_ptr<emcl3d::Emcl3dNode> localization;
  std::shared_ptr<rclcpp::Node> driver;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
};
TEST_F(RosNodeTest, ThreadCountParameterAcceptsAutomaticAndExplicitModes) {
  for (int threads : {0, -3, 1, 4}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"map_path", std::string(TEST_MAP_PATH)}, {"num_threads", threads}});
    const auto node = std::make_shared<emcl3d::Emcl3dNode>(options);
    EXPECT_EQ(node->get_parameter("num_threads").as_int(), threads);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"map_path", std::string(TEST_MAP_PATH)},
    {"num_threads", int64_t(std::numeric_limits<int>::max()) + 1}});
  EXPECT_THROW(std::make_shared<emcl3d::Emcl3dNode>(options), std::invalid_argument);
}
TEST_F(RosNodeTest, LoadsShippedConfigurationIncludingAutomaticSearchBounds) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", TEST_CONFIG_PATH});
  options.parameter_overrides({{"map_path", std::string(TEST_MAP_PATH)}});
  auto configured = std::make_shared<emcl3d::Emcl3dNode>(options);
  EXPECT_EQ(configured->get_parameter("global_localization.search_bounds").as_double_array(),
    (std::vector<double>{0.0, 0.0, 0.0, 0.0}));
}
TEST_F(RosNodeTest, ComputeBackendParameterSelectsExecutionAndRejectsUnavailableModes) {
  for (const std::string backend : {"cpu", "unknown", "vulkan"}) {
    SCOPED_TRACE(backend);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"map_path", std::string(TEST_MAP_PATH)}, {"num_particles", 20},
      {"num_threads", 1}, {"compute_backend", backend}});
    if (backend == "unknown") {
      EXPECT_THROW(std::make_shared<emcl3d::Emcl3dNode>(options), std::invalid_argument);
    } else if (backend == "cpu") {
      EXPECT_NO_THROW(std::make_shared<emcl3d::Emcl3dNode>(options));
    } else {
#ifdef EMCL3D_ENABLE_VULKAN
      EXPECT_NO_THROW(std::make_shared<emcl3d::Emcl3dNode>(options));
#else
      EXPECT_THROW(std::make_shared<emcl3d::Emcl3dNode>(options), std::runtime_error);
#endif
    }
  }
}
TEST_F(RosNodeTest, PublishesObservationStampedSixDimensionalPose) {
  std::promise<geometry_msgs::msg::PoseWithCovarianceStamped> promise;
  auto future = promise.get_future();
  std::promise<geometry_msgs::msg::TransformStamped> transform_promise;
  auto transform_future = transform_promise.get_future();
  bool transform_received = false;
  auto tf_sub = driver->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, [&](const tf2_msgs::msg::TFMessage &message) {
      for (const auto &transform : message.transforms)
        if (!transform_received && transform.header.frame_id == "map" &&
            transform.child_frame_id == "odom") {
          transform_received = true;
          transform_promise.set_value(transform);
        }
    });
  bool received = false;
  auto sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 1, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
      if (!received) { received = true; promise.set_value(message); }
    });
  discover(sub);
  discover(tf_sub);
  auto message = cloud();
  cloud_pub->publish(message);
  await(future);
  ASSERT_TRUE(received);
  const auto result = future.get();
  EXPECT_EQ(result.header.stamp, message.header.stamp);
  EXPECT_EQ(result.header.frame_id, "map");
  EXPECT_NEAR(result.pose.pose.position.z, 0, 1e-10);
  EXPECT_NEAR(std::abs(result.pose.pose.orientation.w), 1, 1e-10);
  EXPECT_NEAR(result.pose.covariance[35], 0, 1e-10);
  await(transform_future);
  ASSERT_TRUE(transform_received);
  const auto correction = transform_future.get();
  EXPECT_EQ(correction.header.stamp, message.header.stamp);
  EXPECT_NEAR(correction.transform.translation.z, -0.7, 1e-10);
}
TEST_F(RosNodeTest, PublishesOutOfMapInitialPoseWithoutAnotherCloud) {
  std::promise<void> ready_promise;
  auto ready = ready_promise.get_future();
  std::promise<geometry_msgs::msg::PoseWithCovarianceStamped> pose_promise;
  auto pose_future = pose_promise.get_future();
  std::promise<geometry_msgs::msg::TransformStamped> tf_promise;
  auto tf_future = tf_promise.get_future();
  bool observed = false, received_pose = false, received_tf = false;
  auto pose_sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &message) {
      if (!observed) {
        observed = true;
        ready_promise.set_value();
      }
      if (!received_pose && message.pose.pose.position.x == 1000.0) {
        received_pose = true;
        pose_promise.set_value(message);
      }
    });
  auto tf_sub = driver->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, [&](const tf2_msgs::msg::TFMessage &message) {
      for (const auto &transform : message.transforms) {
        if (!received_tf && transform.header.frame_id == "map" &&
            transform.child_frame_id == "odom" && transform.transform.translation.x == 1000.0) {
          received_tf = true;
          tf_promise.set_value(transform);
        }
      }
    });
  auto initial_pub = driver->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1);
  discover(pose_sub);
  discover(tf_sub);
  const auto graph = driver->get_graph_event();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (initial_pub->get_subscription_count() == 0) {
    ASSERT_LT(std::chrono::steady_clock::now(), deadline);
    driver->wait_for_graph_change(graph, deadline - std::chrono::steady_clock::now());
    graph->check_and_clear();
  }
  cloud_pub->publish(cloud());
  await(ready);
  ASSERT_TRUE(observed);
  geometry_msgs::msg::PoseWithCovarianceStamped initial;
  initial.header.frame_id = "map";
  initial.header.stamp = driver->now();
  initial.pose.pose.position.x = 1000.0;
  initial.pose.pose.position.y = -1000.0;
  initial.pose.pose.orientation.w = 1.0;
  initial_pub->publish(initial);
  await(pose_future);
  ASSERT_TRUE(received_pose);
  const auto pose = pose_future.get();
  EXPECT_EQ(pose.header.stamp, initial.header.stamp);
  EXPECT_DOUBLE_EQ(pose.pose.pose.position.y, -1000.0);
  await(tf_future);
  ASSERT_TRUE(received_tf);
  const auto transform = tf_future.get();
  EXPECT_EQ(transform.header.stamp, initial.header.stamp);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, -1000.0);
  EXPECT_NEAR(transform.transform.translation.z, -0.7, 1e-10);
}
TEST_F(RosNodeTest, RejectsMalformedStorageWithoutPublishingPose) {
  std::promise<void> promise;
  auto future = promise.get_future();
  bool rejected = false, published_pose = false;
  auto pose_sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 1, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &) {
      published_pose = true;
    });
  auto diagnostic_sub = driver->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 1, [&](const diagnostic_msgs::msg::DiagnosticArray &message) {
      for (const auto &status : message.status)
        for (const auto &value : status.values)
          if (!rejected && value.key == "dropped_clouds" && value.value == "1") {
            rejected = true;
            promise.set_value();
          }
    });
  discover(pose_sub);
  discover(diagnostic_sub);
  auto message = cloud();
  message.data.clear();
  cloud_pub->publish(message);
  await(future);
  EXPECT_TRUE(rejected);
  EXPECT_FALSE(published_pose);
}
TEST_F(RosNodeTest, GlobalActionAppliesMatchedPoseAndReportsObservationStamp) {
  configure_global({{"initial_pose", std::vector<double>{10, 10, 0, 0, 0, 0}}});
  using Action = emcl3d::Emcl3dNode::GlobalAction;
  auto client = rclcpp_action::create_client<Action>(driver, "emcl3d/global_localization");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  std::promise<void> prior_promise;
  auto prior = prior_promise.get_future();
  std::promise<geometry_msgs::msg::PoseWithCovarianceStamped> matched_promise;
  auto matched = matched_promise.get_future();
  bool prior_seen = false, matched_seen = false;
  auto sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
      if (!prior_seen && message.pose.pose.position.x > 5) {
        prior_seen = true;
        prior_promise.set_value();
      }
      if (!matched_seen && message.pose.pose.position.x < 0.1) {
        matched_seen = true;
        matched_promise.set_value(message);
      }
    });
  discover(sub);
  const auto observation = cloud();
  cloud_pub->publish(observation);
  await(prior);

  auto accepted = client->async_send_goal(Action::Goal{});
  await(accepted);
  ASSERT_NE(accepted.get(), nullptr);
  auto result_future = client->async_get_result(accepted.get());
  await(result_future);
  const auto result = result_future.get();
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result->success);
  EXPECT_TRUE(result.result->applied);
  EXPECT_EQ(result.result->pose.header.stamp, observation.header.stamp);
  EXPECT_NEAR(result.result->pose.pose.pose.position.x, 0, 0.03);
  EXPECT_NEAR(result.result->pose.pose.pose.position.y, 0, 0.03);
  await(matched);
  EXPECT_EQ(matched.get().header.stamp, observation.header.stamp);
}

TEST_F(RosNodeTest, GlobalActionEstimateOnlyLeavesTrackingPriorUntouched) {
  configure_global({{"initial_pose", std::vector<double>{10, 10, 0, 0, 0, 0}}});
  using Action = emcl3d::Emcl3dNode::GlobalAction;
  auto client = rclcpp_action::create_client<Action>(driver, "emcl3d/global_localization");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  std::promise<void> observed_promise;
  auto observed = observed_promise.get_future();
  bool observed_once = false;
  auto ready_sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &) {
      if (!observed_once) {observed_once = true; observed_promise.set_value();}
    });
  discover(ready_sub);
  cloud_pub->publish(cloud());
  await(observed);
  Action::Goal goal;
  goal.estimate_only = true;
  auto accepted = client->async_send_goal(goal);
  await(accepted);
  ASSERT_NE(accepted.get(), nullptr);
  auto result_future = client->async_get_result(accepted.get());
  await(result_future);
  ASSERT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(result_future.get().result->applied);

  std::promise<geometry_msgs::msg::PoseWithCovarianceStamped> next_promise;
  auto next = next_promise.get_future();
  bool next_seen = false;
  auto next_cloud = cloud();
  auto next_sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
      if (!next_seen && message.header.stamp == next_cloud.header.stamp) {
        next_seen = true;
        next_promise.set_value(message);
      }
    });
  discover(next_sub);
  cloud_pub->publish(next_cloud);
  await(next);
  EXPECT_GT(next.get().pose.pose.position.x, 5);
}

TEST_F(RosNodeTest, GlobalHandoffAppliesOdometrySinceFrozenObservationExactlyOnce) {
  configure_global({{"initial_pose", std::vector<double>{10, 10, 0, 0, 0, 0}},
    {"odom_frame", "moving_odom"}, {"base_frame", "moving_base"},
    {"global_localization.yaw_range", std::vector<double>{1.5707963267948966, 1.5807963267948966}},
    {"global_localization.num_candidates", 1024}, {"global_localization.batch_size", 16}});
  using Action = emcl3d::Emcl3dNode::GlobalAction;
  auto client = rclcpp_action::create_client<Action>(driver, "emcl3d/global_localization");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  auto motion = driver->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
  discover_publisher(motion);
  const auto send_transform = [&](const geometry_msgs::msg::TransformStamped & transform) {
    tf2_msgs::msg::TFMessage message;
    message.transforms.push_back(transform);
    motion->publish(message);
  };
  std::promise<void> prior_promise, searching_promise;
  auto prior = prior_promise.get_future();
  auto searching = searching_promise.get_future();
  std::promise<geometry_msgs::msg::PoseWithCovarianceStamped> applied_promise;
  auto applied = applied_promise.get_future();
  bool prior_seen = false, searching_seen = false, applied_seen = false;
  auto sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
      if (!prior_seen && message.pose.pose.position.x > 5) {
        prior_seen = true;
        prior_promise.set_value();
      }
      if (!applied_seen && message.pose.pose.position.x < 1) {
        applied_seen = true;
        applied_promise.set_value(message);
      }
    });
  discover(sub);
  auto first = cloud();
  first.header.frame_id = "moving_base";
  sensor_msgs::PointCloud2Iterator<float> first_x(first, "x"), first_y(first, "y");
  for (; first_x != first_x.end(); ++first_x, ++first_y) {
    const float x = *first_x;
    *first_x = *first_y;
    *first_y = -x;
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "moving_odom";
  transform.child_frame_id = "moving_base";
  transform.header.stamp = first.header.stamp;
  transform.transform.rotation.w = 1;
  send_transform(transform);
  cloud_pub->publish(first);
  await(prior);
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [&](auto, auto feedback) {
    if (!searching_seen && feedback->evaluated_candidates > 0) {
      searching_seen = true;
      searching_promise.set_value();
    }
  };
  auto accepted = client->async_send_goal(Action::Goal{}, options);
  await(accepted);
  ASSERT_NE(accepted.get(), nullptr);
  auto result = client->async_get_result(accepted.get());
  await(searching);

  auto second = first;
  second.header.stamp = driver->now();
  for (sensor_msgs::PointCloud2Iterator<float> x(second, "x"); x != x.end(); ++x) {
    *x -= 0.5f;
  }
  transform.header.stamp = second.header.stamp;
  transform.transform.translation.x = 0.5;
  send_transform(transform);
  cloud_pub->publish(second);
  await(result);
  ASSERT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  await(applied);
  const auto applied_pose = applied.get();
  EXPECT_EQ(result.get().result->pose.header.stamp, first.header.stamp);
  EXPECT_EQ(applied_pose.header.stamp, second.header.stamp);
  EXPECT_NEAR(applied_pose.pose.pose.position.x, 0, 0.03);
  EXPECT_NEAR(applied_pose.pose.pose.position.y, 0.5, 0.03);

  std::promise<geometry_msgs::msg::PoseWithCovarianceStamped> next_promise;
  auto next = next_promise.get_future();
  auto third = second;
  third.header.stamp = driver->now();
  bool next_seen = false;
  auto next_sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
      if (!next_seen && message.header.stamp == third.header.stamp) {
        next_seen = true;
        next_promise.set_value(message);
      }
    });
  discover(next_sub);
  transform.header.stamp = third.header.stamp;
  send_transform(transform);
  cloud_pub->publish(third);
  await(next);
  const auto next_pose = next.get();
  EXPECT_NEAR(next_pose.pose.pose.position.x, 0, 0.03);
  EXPECT_NEAR(next_pose.pose.pose.position.y, 0.5, 0.03);
}

TEST_F(RosNodeTest, GlobalActionCanBeCanceledWhileSearchingAndRejectsConcurrentGoals) {
  configure_global({{"global_localization.num_candidates", 100000},
    {"global_localization.batch_size", 1}});
  using Action = emcl3d::Emcl3dNode::GlobalAction;
  auto client = rclcpp_action::create_client<Action>(driver, "emcl3d/global_localization");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  std::promise<void> waiting_promise;
  auto waiting = waiting_promise.get_future();
  bool waiting_seen = false;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [&](auto, auto feedback) {
    if (!waiting_seen && feedback->evaluated_candidates > 0) {
      waiting_seen = true;
      waiting_promise.set_value();
    }
  };
  auto accepted = client->async_send_goal(Action::Goal{}, options);
  await(accepted);
  ASSERT_NE(accepted.get(), nullptr);
  auto result_future = client->async_get_result(accepted.get());
  cloud_pub->publish(cloud());
  await(waiting);
  auto second = client->async_send_goal(Action::Goal{});
  await(second);
  EXPECT_EQ(second.get(), nullptr);
  auto cancellation = client->async_cancel_goal(accepted.get());
  await(cancellation);
  await(result_future);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result_future.get().result->code, "CANCELED");
}

TEST_F(RosNodeTest, GlobalActionUsesPublishedFreeSpaceInsteadOfInferringItFromPcd) {
  configure_global({{"global_localization.free_cells_only", true}});
  using Action = emcl3d::Emcl3dNode::GlobalAction;
  auto client = rclcpp_action::create_client<Action>(driver, "emcl3d/global_localization");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  auto grid_pub = driver->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).reliable().transient_local());
  auto accepted = client->async_send_goal(Action::Goal{});
  await(accepted);
  ASSERT_NE(accepted.get(), nullptr);
  auto result_future = client->async_get_result(accepted.get());
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 1;
  grid.info.width = grid.info.height = 4;
  grid.info.origin.position.x = grid.info.origin.position.y = -1;
  grid.info.origin.orientation.w = 1;
  grid.data.assign(16, -1);
  grid_pub->publish(grid);
  cloud_pub->publish(cloud());
  await(result_future);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result_future.get().result->code, "no_valid_candidates");
  std::promise<void> grid_updated_promise;
  auto grid_updated = grid_updated_promise.get_future();
  bool grid_updated_seen = false;
  auto diagnostics = driver->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray & message) {
      for (const auto & entry : message.status) {
        for (const auto & value : entry.values) {
          if (!grid_updated_seen && value.key == "global_localization_grid_updates" && value.value == "2") {
            grid_updated_seen = true;
            grid_updated_promise.set_value();
          }
        }
      }
    });
  discover(diagnostics);
  grid.data.assign(16, 0);
  grid_pub->publish(grid);
  await(grid_updated);
  auto retry = client->async_send_goal(Action::Goal{});
  await(retry);
  ASSERT_NE(retry.get(), nullptr);
  auto retry_result = client->async_get_result(retry.get());
  await(retry_result);
  EXPECT_EQ(retry_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(retry_result.get().result->applied);
}

TEST_F(RosNodeTest, ValidManualInitialPoseAbortsGlobalSearch) {
  configure_global({{"global_localization.num_candidates", 100000},
    {"global_localization.batch_size", 1}});
  using Action = emcl3d::Emcl3dNode::GlobalAction;
  auto client = rclcpp_action::create_client<Action>(driver, "emcl3d/global_localization");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  auto initial_pub = driver->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1);
  discover_publisher(initial_pub);
  std::promise<void> ready_promise;
  auto ready = ready_promise.get_future();
  bool ready_seen = false;
  auto ready_sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 1, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &) {
      if (!ready_seen) {ready_seen = true; ready_promise.set_value();}
    });
  discover(ready_sub);
  cloud_pub->publish(cloud());
  await(ready);
  auto accepted = client->async_send_goal(Action::Goal{});
  await(accepted);
  ASSERT_NE(accepted.get(), nullptr);
  auto result_future = client->async_get_result(accepted.get());
  geometry_msgs::msg::PoseWithCovarianceStamped initial;
  initial.header.frame_id = "map";
  initial.header.stamp = driver->now();
  initial.pose.pose.orientation.w = 1;
  initial.pose.pose.position.x = 100;
  initial_pub->publish(initial);
  await(result_future);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result_future.get().result->code, "MANUAL_INITIALPOSE");
}

TEST_F(RosNodeTest, GlobalSearchTimeoutAbortsWithoutApplyingPartialResult) {
  configure_global({{"global_localization.num_candidates", 100000},
    {"global_localization.batch_size", 1}, {"global_localization.timeout", 0.001}});
  using Action = emcl3d::Emcl3dNode::GlobalAction;
  auto client = rclcpp_action::create_client<Action>(driver, "emcl3d/global_localization");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  auto accepted = client->async_send_goal(Action::Goal{});
  await(accepted);
  ASSERT_NE(accepted.get(), nullptr);
  auto result = client->async_get_result(accepted.get());
  cloud_pub->publish(cloud());
  await(result);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->code, "TIMEOUT");
  EXPECT_FALSE(result.get().result->applied);
}

TEST_F(RosNodeTest, StartupLocalizationWaitsForCloudThenAppliesOnce) {
  configure_global({{"global_localization.run_on_startup", true},
    {"initial_pose", std::vector<double>{10, 10, 0, 0, 0, 0}}});
  std::promise<geometry_msgs::msg::PoseWithCovarianceStamped> pose_promise;
  auto pose = pose_promise.get_future();
  bool pose_seen = false;
  auto sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
      if (!pose_seen) {pose_seen = true; pose_promise.set_value(message);}
    });
  discover(sub);
  cloud_pub->publish(cloud());
  await(pose);
  const auto result = pose.get();
  EXPECT_NEAR(result.pose.pose.position.x, 0, 0.03);
  EXPECT_NEAR(result.pose.pose.position.y, 0, 0.03);
}

TEST_F(RosNodeTest, StartupInputTimeoutDoesNotPublishConfiguredPrior) {
  configure_global({{"global_localization.run_on_startup", true},
    {"global_localization.startup_wait_timeout", 0.03}});
  std::promise<void> timeout_promise;
  auto timed_out = timeout_promise.get_future();
  bool reported = false, published = false;
  auto pose_sub = driver->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose", 10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped &) {published = true;});
  auto diagnostic_sub = driver->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray & message) {
      for (const auto & entry : message.status) {
        for (const auto & value : entry.values) {
          if (!reported && value.key == "global_localization_code" && value.value == "NO_CLOUD") {
            reported = true;
            timeout_promise.set_value();
          }
        }
      }
    });
  discover(pose_sub);
  discover(diagnostic_sub);
  await(timed_out);
  EXPECT_FALSE(published);
}
}  // namespace
