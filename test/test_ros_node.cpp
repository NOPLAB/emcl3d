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
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/emcl3d_test"});
    options.parameter_overrides({
      {"map_path", std::string(TEST_MAP_PATH)}, {"num_particles", 20}, {"num_threads", 1},
      {"min_valid_points", 4}, {"scan_voxel_size", 0.01},
      {"map_voxel_size", 0.01}, {"max_cloud_age", 30.0},
      {"future_tolerance", 10.0},
      {"initial_variance", std::vector<double>(6, 0.0)},
      {"translation_noise", std::vector<double>(3, 0.0)},
      {"rotation_noise", std::vector<double>(3, 0.0)}});
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
  template<typename T> void await(std::future<T> &future) {
    ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
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
}  // namespace
