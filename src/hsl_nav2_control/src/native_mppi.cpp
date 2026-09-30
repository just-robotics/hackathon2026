// SPDX-License-Identifier: Apache-2.0
// Project wrapper around the unmodified Nav2 Humble MPPIController library.
// Nav2 implementation and authors: docs/NAV2_MPPI_ADAPTATION.md.
#include <algorithm>
#include <array>
#include <map>
#include <mutex>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "hsl_interfaces/msg/planning_intent.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "xtensor/xrandom.hpp"

class VisibleMPPI : public nav2_mppi_controller::MPPIController
{
public:
  auto trajectory() {return optimizer_.getOptimizedTrajectory();}
};

class NativeMPPI : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NativeMPPI(const rclcpp::NodeOptions & options)
  : LifecycleNode("native_mppi", options) {}

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    nav2_util::declare_parameter_if_not_declared(shared_from_this(), "role",
      rclcpp::ParameterValue("explorer"));
    nav2_util::declare_parameter_if_not_declared(shared_from_this(), "random_seed",
      rclcpp::ParameterValue(0));
    nav2_util::declare_parameter_if_not_declared(shared_from_this(), "local_path_length",
      rclcpp::ParameterValue(1.2));
    role_ = get_parameter("role").as_string();
    if (role_ != "explorer" && role_ != "guardian") {
      throw std::invalid_argument("role must be explorer or guardian");
    }
    const auto ns = std::string(get_namespace());
    const auto prefix = ns == "/" ? std::string{} : ns;
    std::vector<rclcpp::Parameter> overrides;
    for (const auto & name : list_parameters({"costmap"}, 0).names) {
      const auto local_name = name.substr(std::string("costmap.").size());
      if (local_name != "robot_radius") {
        overrides.emplace_back(local_name, get_parameter(name).get_parameter_value());
      }
    }
    // Use the same protected body envelope as the global planner. The
    // obstacle critic still scores proximity outside the collision footprint.
    const double safety_radius = get_parameter("costmap.robot_radius").as_double();
    if (!std::isfinite(safety_radius) || safety_radius <= 0.0) {
      throw std::invalid_argument("costmap.robot_radius must be finite and positive");
    }
    overrides.emplace_back("robot_radius", safety_radius);
    // Circumscribed polygon preserves the full disk clearance while satisfying
    // the official footprint critic's explicit-shape requirement.
    constexpr int vertices = 32;
    const double polygon_radius = safety_radius / std::cos(M_PI / vertices);
    std::ostringstream footprint;
    footprint << "[";
    for (int i = 0; i < vertices; ++i) {
      if (i) {footprint << ",";}
      const double angle = 2 * M_PI * i / vertices;
      footprint << "[" << polygon_radius * std::cos(angle) << ","
        << polygon_radius * std::sin(angle) << "]";
    }
    footprint << "]";
    overrides.emplace_back("footprint", footprint.str());
    overrides.emplace_back("use_sim_time", get_parameter("use_sim_time").as_bool());
    overrides.emplace_back("robot_base_frame", prefix.empty() ? "base_footprint" :
      prefix.substr(1) + "/base_footprint");
    overrides.emplace_back("obstacle_layer.cloud.sensor_frame",
      prefix.empty() ? "base_footprint" : prefix.substr(1) + "/base_footprint");
    overrides.emplace_back("obstacle_layer.cloud.topic", prefix + "/navigation/nav2_scan");
    rclcpp::NodeOptions costmap_options;
    costmap_options.use_global_arguments(false).parameter_overrides(overrides).arguments(
      {"--ros-args", "-r", "__node:=native_costmap", "-r",
        "__ns:=" + prefix + "/native_mppi"});
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_);
    if (costmap_->configure().label() != "inactive") {
      throw std::runtime_error("Nav2 costmap configuration failed");
    }
    xt::random::seed(static_cast<unsigned int>(get_parameter("random_seed").as_int()));
    controller_.configure(shared_from_this(), "MPPI", costmap_->getTfBuffer(), costmap_);
    goal_checker_.initialize(shared_from_this(), "goal_checker", costmap_);
    auto state_qos = rclcpp::QoS(1).transient_local().reliable();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("navigation/mppi_cmd_vel", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("navigation/local_path", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("navigation/planner_status", 10);
    diag_pub_ = create_publisher<std_msgs::msg::String>("navigation/mppi_diagnostics", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "navigation/nav2_scan", rclcpp::SensorDataQoS());
    timing_pub_ = create_publisher<std_msgs::msg::Float32>(
      "navigation/native_mppi_cycle_ms", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("navigation/native_ready", state_qos);
    subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "navigation/self", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {own_ = msg;}));
    subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "navigation/opponent", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {opponent_ = msg;}));
    subscriptions_.push_back(create_subscription<hsl_interfaces::msg::PlanningIntent>(
      "navigation/intent", 10,
      [this](hsl_interfaces::msg::PlanningIntent::SharedPtr msg) {intent_ = msg;}));
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
      "navigation/global_status", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {global_status_ = msg->data;}));
    subscriptions_.push_back(create_subscription<nav_msgs::msg::Path>(
      "navigation/nav2_reference", 10, [this](nav_msgs::msg::Path::SharedPtr msg) {
        reference_ = msg;
        if (msg->poses.size() >= 2) {controller_.setPlan(*msg);}
      }));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
      "navigation/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {filter_scan(*msg);}));
    const double frequency = get_parameter("controller_frequency").as_double();
    if (frequency <= 0.0) {throw std::invalid_argument("controller_frequency must be positive");}
    timer_ = rclcpp::create_timer(this, get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / frequency)), [this]() {tick();});
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    cmd_pub_->on_activate(); path_pub_->on_activate(); status_pub_->on_activate();
    diag_pub_->on_activate(); scan_pub_->on_activate(); timing_pub_->on_activate();
    ready_pub_->on_activate();
    if (costmap_->activate().label() != "active") {
      throw std::runtime_error("Nav2 costmap activation failed");
    }
    controller_.activate();
    active_ = true;
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    publish_stop("WAIT_OR_STOP");
    controller_.deactivate();
    costmap_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    subscriptions_.clear();
    controller_.cleanup();
    costmap_->cleanup();
    costmap_thread_.reset();
    costmap_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  bool fresh(const builtin_interfaces::msg::Time & stamp, double timeout) const
  {
    const auto age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= 0.0 && age <= timeout;
  }

  void filter_scan(const sensor_msgs::msg::PointCloud2 & source)
  {
    scan_stamp_ = source.header.stamp;
    sensor_msgs::msg::PointCloud2 result;
    result.header = source.header;
    sensor_msgs::PointCloud2Modifier modifier(result);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    const bool omit_opponent = role_ == "guardian" && opponent_ &&
      fresh(opponent_->header.stamp, 2.0);
    std::vector<std::array<float, 3>> points;
    sensor_msgs::PointCloud2ConstIterator<float> x(source, "x"), y(source, "y"), z(source, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
      if (omit_opponent && std::hypot(*x - opponent_->pose.pose.position.x,
          *y - opponent_->pose.pose.position.y) <= 0.45) {continue;}
      points.push_back({*x, *y, *z});
    }
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> rx(result, "x"), ry(result, "y"), rz(result, "z");
    for (const auto & p : points) { *rx = p[0]; *ry = p[1]; *rz = p[2]; ++rx; ++ry; ++rz; }
    scan_pub_->publish(result);
  }

  void publish_stop(const std::string & status, const std::string & reason = "")
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map"; empty.header.stamp = now();
    path_pub_->publish(empty);
    std_msgs::msg::String state; state.data = status; status_pub_->publish(state);
    std::ostringstream json;
    json << "{\"backend\":\"nav2_cpp\",\"result\":\""
      << (reason.empty() ? status : reason) << "\",\"recovery\":"
      << (global_status_ == "RECOVERY_ROUTE" ? "true" : "false")
      << ",\"first_speed_mps\":0,\"first_omega_radps\":0";
    if (reason == "swept_collision") {
      json << ",\"rejected_x_m\":" << rejected_x_
        << ",\"rejected_y_m\":" << rejected_y_
        << ",\"rejected_cost\":" << rejected_cost_
        << ",\"rejected_trajectory_index\":" << rejected_index_;
    }
    json << "}";
    std_msgs::msg::String diag; diag.data = json.str(); diag_pub_->publish(diag);
  }

  bool swept_safe(const nav_msgs::msg::Path & path)
  {
    auto * grid = costmap_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(grid);
    const auto footprint = costmap_->getRobotFootprint();
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose;
      const auto & b = path.poses[i].pose;
      const double yaw = tf2::getYaw(a.orientation);
      const double dyaw = std::remainder(tf2::getYaw(b.orientation) - yaw, 2 * M_PI);
      const double distance = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
      const int samples = std::max(1, static_cast<int>(std::ceil(
        std::max(distance / (grid->getResolution() * 0.5), std::abs(dyaw) / 0.05))));
      for (int j = 0; j <= samples; ++j) {
        const double fraction = static_cast<double>(j) / samples;
        const double cost = checker.footprintCostAtPose(
          a.position.x + fraction * (b.position.x - a.position.x),
          a.position.y + fraction * (b.position.y - a.position.y),
          yaw + fraction * dyaw, footprint);
        if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          rejected_x_ = a.position.x + fraction * (b.position.x - a.position.x);
          rejected_y_ = a.position.y + fraction * (b.position.y - a.position.y);
          rejected_cost_ = cost;
          rejected_index_ = i;
          return false;
        }
      }
    }
    return true;
  }

  void tick()
  {
    if (!active_) {return;}
    const auto started = std::chrono::steady_clock::now();
    tick_control();
    std_msgs::msg::Float32 timing;
    timing.data = std::chrono::duration<float, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    timing_pub_->publish(timing);
  }

  void tick_control()
  {
    std_msgs::msg::Bool ready;
    ready.data = own_ && fresh(own_->header.stamp, 1.2) && fresh(scan_stamp_, 1.8) &&
      costmap_->isCurrent();
    ready_pub_->publish(ready);
    if (!intent_ || intent_->behavior <= 1) {publish_stop("WAIT_OR_STOP"); return;}
    if (!ready.data || !fresh(intent_->header.stamp, 1.0)) {
      publish_stop("STALE_INPUT"); return;
    }
    if (!reference_ || reference_->poses.size() < 2 ||
      !fresh(reference_->header.stamp, 1.0)) {
      publish_stop(global_status_ == "OK" || global_status_ == "RECOVERY_ROUTE" ||
        global_status_.empty() ? "NO_GLOBAL_PATH" : global_status_); return;
    }
    if (own_->header.frame_id != "map" || reference_->header.frame_id != "map") {
      publish_stop("BAD_FRAME"); return;
    }
    try {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = own_->header; pose.pose = own_->pose.pose;
      auto cmd = controller_.computeVelocityCommands(pose, own_->twist.twist, &goal_checker_);
      const auto trajectory = controller_.trajectory();
      nav_msgs::msg::Path path;
      path.header.frame_id = "map"; path.header.stamp = now();
      pose.header = path.header; path.poses.push_back(pose);
      for (size_t i = 0; i < trajectory.shape()[0]; ++i) {
        geometry_msgs::msg::PoseStamped point;
        point.header = path.header;
        point.pose.position.x = trajectory(i, 0); point.pose.position.y = trajectory(i, 1);
        tf2::Quaternion quaternion; quaternion.setRPY(0, 0, trajectory(i, 2));
        point.pose.orientation = tf2::toMsg(quaternion);
        path.poses.push_back(point);
      }
      if (!swept_safe(path)) {publish_stop("NO_LOCAL_PATH", "swept_collision"); return;}
      nav_msgs::msg::Path prefix;
      prefix.header = path.header; prefix.poses.push_back(path.poses.front());
      double length = 0.0;
      for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto & a = path.poses[i - 1].pose.position;
        const auto & b = path.poses[i].pose.position;
        const double step = std::hypot(b.x - a.x, b.y - a.y);
        if (length + step > get_parameter("local_path_length").as_double()) {break;}
        prefix.poses.push_back(path.poses[i]); length += step;
      }
      const bool recovery = global_status_ == "RECOVERY_ROUTE";
      path_pub_->publish(prefix); cmd_pub_->publish(cmd.twist);
      std_msgs::msg::String state; state.data = recovery ? "RECOVERY_MPPI" : "OK";
      status_pub_->publish(state);
      std::ostringstream json;
      json << "{\"backend\":\"nav2_cpp\",\"result\":\"ok\",\"recovery\":"
        << (recovery ? "true" : "false") << ",\"first_speed_mps\":" << cmd.twist.linear.x
        << ",\"first_omega_radps\":" << cmd.twist.angular.z << "}";
      std_msgs::msg::String diag; diag.data = json.str(); diag_pub_->publish(diag);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPPI: %s", error.what());
      publish_stop("NO_LOCAL_PATH", "optimizer_failure");
    }
  }

  VisibleMPPI controller_;
  nav2_controller::SimpleGoalChecker goal_checker_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::SharedPtr own_, opponent_;
  nav_msgs::msg::Path::SharedPtr reference_;
  hsl_interfaces::msg::PlanningIntent::SharedPtr intent_;
  builtin_interfaces::msg::Time scan_stamp_;
  double rejected_x_{0.0}, rejected_y_{0.0}, rejected_cost_{0.0};
  size_t rejected_index_{0};
  std::string role_, global_status_;
  bool active_{false};
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_, diag_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr timing_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NativeMPPI>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (node->configure().label() != "inactive" ||
    node->activate().label() != "active") {
    RCLCPP_ERROR(node->get_logger(), "Native MPPI lifecycle startup failed");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
