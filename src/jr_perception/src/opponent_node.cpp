#include "jr_perception/opponent_core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace core = jr_perception::opponent_core;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool valid_quaternion(const tf2::Quaternion & q)
{
  const double n = q.length2();
  return std::isfinite(n) && n > 0.98 && n < 1.02;
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string json_quote(const std::string & input)
{
  std::string out = "\"";
  for (char c : input) {
    if (c == '\"' || c == '\\') {out += '\\';}
    if (static_cast<unsigned char>(c) < 32) {out += ' ';} else {out += c;}
  }
  return out + '\"';
}

float read_float32(const std::uint8_t * data, bool big_endian)
{
  std::array<std::uint8_t, 4> bytes{};
  const std::uint16_t word = 1;
  const bool host_big_endian = *reinterpret_cast<const std::uint8_t *>(&word) == 0;
  for (std::size_t i = 0; i < 4; ++i) {
    bytes[i] = data[big_endian == host_big_endian ? i : 3 - i];
  }
  float value;
  std::memcpy(&value, bytes.data(), 4);
  return value;
}

double read_float64(const std::uint8_t * data, bool big_endian)
{
  std::array<std::uint8_t, 8> bytes{};
  const std::uint16_t word = 1;
  const bool host_big_endian = *reinterpret_cast<const std::uint8_t *>(&word) == 0;
  for (std::size_t i = 0; i < 8; ++i) {
    bytes[i] = data[big_endian == host_big_endian ? i : 7 - i];
  }
  double value;
  std::memcpy(&value, bytes.data(), 8);
  return value;
}

struct CloudFields
{
  int x = -1;
  int y = -1;
  int z = -1;
  int intensity = -1;
  int tag = -1;
  int line = -1;
  int timestamp = -1;
};

bool field_valid(const PointField & field, std::size_t bytes, std::size_t step)
{
  return field.count == 1 && field.offset <= step && bytes <= step - field.offset;
}

std::optional<CloudFields> cloud_fields(const PointCloud2 & cloud)
{
  CloudFields result;
  for (const auto & field : cloud.fields) {
    if (!field_valid(field, field.datatype == PointField::FLOAT64 ? 8 :
      field.datatype == PointField::UINT8 ? 1 : 4, cloud.point_step))
    {
      continue;
    }
    const int offset = static_cast<int>(field.offset);
    if (field.name == "x" && field.datatype == PointField::FLOAT32) {result.x = offset;}
    if (field.name == "y" && field.datatype == PointField::FLOAT32) {result.y = offset;}
    if (field.name == "z" && field.datatype == PointField::FLOAT32) {result.z = offset;}
    if (field.name == "intensity" && field.datatype == PointField::FLOAT32) {
      result.intensity = offset;
    }
    if (field.name == "tag" && field.datatype == PointField::UINT8) {result.tag = offset;}
    if (field.name == "line" && field.datatype == PointField::UINT8) {result.line = offset;}
    if (field.name == "timestamp" && field.datatype == PointField::FLOAT64) {
      result.timestamp = offset;
    }
  }
  if (result.x < 0 || result.y < 0 || result.z < 0) {return std::nullopt;}
  return result;
}

struct StampedPose
{
  double stamp = 0.0;
  double bracket_gap_s = 0.0;
  std::string selection = "exact";
  tf2::Vector3 position;
  tf2::Quaternion orientation;
  double var_x = 0.0;
  double var_y = 0.0;
  double var_yaw = 0.0;
};

struct ParsedCloud
{
  std::vector<core::Point> points;
  std::vector<double> point_stamps_s;
  double packet_span_s = 0.0;
  double first_point_stamp_s = 0.0;
  double last_point_stamp_s = 0.0;
  std::size_t invalid_points = 0;
};

std::string parse_cloud(const PointCloud2 & cloud, std::size_t max_points,
  bool require_timestamps, ParsedCloud & parsed)
{
  const auto fields = cloud_fields(cloud);
  if (!fields) {return "invalid_xyz_fields";}
  if (require_timestamps && fields->timestamp < 0) {return "missing_point_timestamps";}
  if (cloud.height == 0 || cloud.width == 0 || cloud.point_step == 0 ||
    cloud.width > max_points / cloud.height ||
    static_cast<std::size_t>(cloud.width) * cloud.point_step > cloud.row_step ||
    cloud.data.size() < static_cast<std::size_t>(cloud.height - 1) * cloud.row_step +
    static_cast<std::size_t>(cloud.width) * cloud.point_step)
  {
    return "invalid_cloud_layout_or_size";
  }
  parsed.points.reserve(static_cast<std::size_t>(cloud.height) * cloud.width);
  if (require_timestamps) {
    parsed.point_stamps_s.reserve(static_cast<std::size_t>(cloud.height) * cloud.width);
  }
  double min_stamp = std::numeric_limits<double>::infinity();
  double max_stamp = -std::numeric_limits<double>::infinity();
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const auto * point = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(col) * cloud.point_step;
      core::Point out;
      out.x = read_float32(point + fields->x, cloud.is_bigendian);
      out.y = read_float32(point + fields->y, cloud.is_bigendian);
      out.z = read_float32(point + fields->z, cloud.is_bigendian);
      if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.z)) {
        ++parsed.invalid_points;
        continue;
      }
      if (fields->intensity >= 0) {
        out.intensity = read_float32(point + fields->intensity, cloud.is_bigendian);
      }
      if (fields->tag >= 0) {out.tag = point[fields->tag];}
      if (fields->line >= 0) {out.line = point[fields->line];}
      if (fields->timestamp >= 0) {
        const double ns = read_float64(point + fields->timestamp, cloud.is_bigendian);
        // Livox Mid-360 stores an absolute nanosecond timestamp in FLOAT64.
        if (std::isfinite(ns) && ns > 1e15 && ns < 1e20) {
          min_stamp = std::min(min_stamp, ns);
          max_stamp = std::max(max_stamp, ns);
          if (require_timestamps) {parsed.point_stamps_s.push_back(ns * 1e-9);}
        } else if (require_timestamps) {
          return "invalid_point_timestamps";
        }
      }
      parsed.points.push_back(out);
    }
  }
  if (std::isfinite(min_stamp) && std::isfinite(max_stamp)) {
    parsed.packet_span_s = (max_stamp - min_stamp) * 1e-9;
    parsed.first_point_stamp_s = min_stamp * 1e-9;
    parsed.last_point_stamp_s = max_stamp * 1e-9;
  }
  return parsed.points.empty() ? "no_finite_points" : "";
}

tf2::Quaternion quaternion_of(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::Quaternion(q.x, q.y, q.z, q.w);
}

}  // namespace

class OpponentNode final : public rclcpp::Node
{
public:
  OpponentNode()
  : Node("opponent_detector_cpp"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, true)
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/sensing/lidar/points_filtered");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/kinematic_state");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    allow_mapless_ = declare_parameter<bool>("allow_mapless", false);
    if (allow_mapless_ && world_frame_ != "odom") {
      throw std::invalid_argument("mapless replay requires explicit world_frame=odom");
    }
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    opponent_frame_ = declare_parameter<std::string>("opponent_frame", "opponent_tracking_frame");
    max_cloud_age_s_ = declare_parameter<double>("max_cloud_age_s", 0.8);
    max_future_s_ = declare_parameter<double>("max_future_s", 0.20);
    max_pose_gap_s_ = declare_parameter<double>("max_pose_gap_s", 0.25);
    pose_nearest_tolerance_s_ = declare_parameter<double>("pose_nearest_tolerance_s", 0.025);
    pose_history_s_ = declare_parameter<double>("pose_history_s", 3.0);
    max_position_std_ = declare_parameter<double>("max_position_std", 0.50);
    max_yaw_std_ = declare_parameter<double>("max_yaw_std", 0.70);
    visible_timeout_s_ = declare_parameter<double>("visible_timeout_s", 0.25);
    const int max_cloud_points = declare_parameter<int>("max_cloud_points", 150000);
    const int max_marker_candidates = declare_parameter<int>("max_marker_candidates", 24);
    const int max_marker_tracks = declare_parameter<int>("max_marker_tracks", 8);
    const int max_marker_points = declare_parameter<int>("max_marker_points", 256);
    if (max_cloud_points <= 0 || max_marker_candidates <= 0 ||
      max_marker_tracks <= 0 || max_marker_points <= 0)
    {
      throw std::invalid_argument("invalid opponent detector size parameter");
    }
    max_cloud_points_ = static_cast<std::size_t>(max_cloud_points);
    max_marker_candidates_ = static_cast<std::size_t>(max_marker_candidates);
    max_marker_tracks_ = static_cast<std::size_t>(max_marker_tracks);
    max_marker_points_ = static_cast<std::size_t>(max_marker_points);
    if (!std::isfinite(max_cloud_age_s_) || !std::isfinite(max_future_s_) ||
      !std::isfinite(max_pose_gap_s_) || !std::isfinite(pose_nearest_tolerance_s_) ||
      !std::isfinite(pose_history_s_) || !std::isfinite(max_position_std_) ||
      !std::isfinite(max_yaw_std_) || !std::isfinite(visible_timeout_s_) ||
      max_cloud_age_s_ <= 0 || max_future_s_ < 0 || max_pose_gap_s_ <= 0 ||
      pose_nearest_tolerance_s_ < 0 ||
      pose_history_s_ < max_pose_gap_s_ || max_position_std_ <= 0 || max_yaw_std_ <= 0 ||
      visible_timeout_s_ <= 0 || max_cloud_points_ == 0 || max_marker_candidates_ == 0 ||
      max_marker_tracks_ == 0 || max_marker_points_ == 0)
    {
      throw std::invalid_argument("invalid opponent detector ROS parameter");
    }
    detector_ = std::make_unique<core::Detector>(detector_config());
    odom_pub_ = create_publisher<Odometry>("opponent/odom", 10);
    markers_pub_ = create_publisher<MarkerArray>("opponent/markers", 10);
    visible_pub_ = create_publisher<std_msgs::msg::Bool>("navigation/opponent_visible", 10);
    diagnostics_pub_ = create_publisher<std_msgs::msg::String>("navigation/detector_diagnostics", 10);
    map_sub_ = create_subscription<OccupancyGrid>(map_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](OccupancyGrid::SharedPtr msg) {on_map(*msg);});
    pose_sub_ = create_subscription<Odometry>(pose_topic_, rclcpp::QoS(100).reliable(),
      [this](Odometry::SharedPtr msg) {on_pose(*msg);});
    cloud_sub_ = create_subscription<PointCloud2>(cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](PointCloud2::SharedPtr msg) {on_cloud(std::move(msg));});
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "C++ opponent detector: cloud=%s pose=%s map=%s",
      cloud_topic_.c_str(), pose_topic_.c_str(), map_topic_.c_str());
  }

private:
  core::Config detector_config()
  {
    core::Config config;
    config.allow_mapless = allow_mapless_;
    config.min_range = declare_parameter<double>("min_range", config.min_range);
    config.max_range = declare_parameter<double>("max_range", config.max_range);
    config.min_height = declare_parameter<double>("min_height", config.min_height);
    config.max_height = declare_parameter<double>("max_height", config.max_height);
    config.voxel_size = declare_parameter<double>("voxel_size", config.voxel_size);
    config.cluster_gap = declare_parameter<double>("cluster_gap", config.cluster_gap);
    config.robot_radius = declare_parameter<double>("robot_radius", config.robot_radius);
    config.min_map_alignment_share = declare_parameter<double>("min_map_alignment_share",
      config.min_map_alignment_share);
    const int min_map_alignment_points = declare_parameter<int>("min_map_alignment_points",
      static_cast<int>(config.min_map_alignment_points));
    if (min_map_alignment_points <= 0) {
      throw std::invalid_argument("nonpositive min_map_alignment_points");
    }
    config.min_map_alignment_points = static_cast<std::size_t>(min_map_alignment_points);
    config.max_track_gap_s = declare_parameter<double>("max_track_gap_s", config.max_track_gap_s);
    config.max_track_age_s = declare_parameter<double>("max_track_age_s", config.max_track_age_s);
    config.confirmation_hits = declare_parameter<int>("confirmation_hits", config.confirmation_hits);
    const int core_marker_points = declare_parameter<int>("core_marker_points",
      static_cast<int>(config.max_marker_points));
    if (core_marker_points < 0) {throw std::invalid_argument("negative core_marker_points");}
    config.max_marker_points = static_cast<std::size_t>(core_marker_points);
    if (!std::isfinite(config.min_range) || !std::isfinite(config.max_range) ||
      !std::isfinite(config.min_height) || !std::isfinite(config.max_height) ||
      !std::isfinite(config.voxel_size) || !std::isfinite(config.cluster_gap) ||
      !std::isfinite(config.robot_radius) || !std::isfinite(config.max_track_gap_s) ||
      !std::isfinite(config.min_map_alignment_share) ||
      config.min_map_alignment_share < 0 || config.min_map_alignment_share > 1 ||
      !std::isfinite(config.max_track_age_s) || config.min_range < 0 ||
      config.max_range <= config.min_range || config.max_height <= config.min_height ||
      config.voxel_size <= 0 || config.cluster_gap <= 0 || config.robot_radius <= 0 ||
      config.max_track_gap_s <= 0 || config.max_track_age_s <= 0 ||
      config.confirmation_hits <= 0)
    {
      throw std::invalid_argument("invalid opponent detector core parameter");
    }
    return config;
  }

  void on_map(const OccupancyGrid & msg)
  {
    const auto & info = msg.info;
    if (msg.header.frame_id != world_frame_ || info.width == 0 || info.height == 0 ||
      !std::isfinite(info.resolution) || info.resolution <= 0 ||
      static_cast<std::size_t>(info.width) * info.height != msg.data.size())
    {
      status_ = "invalid_map";
      map_.reset();
      return;
    }
    const auto & origin = info.origin;
    const auto q = quaternion_of(origin.orientation);
    if (!valid_quaternion(q) || !std::isfinite(origin.position.x) ||
      !std::isfinite(origin.position.y))
    {
      status_ = "invalid_map_origin";
      map_.reset();
      return;
    }
    core::Map next;
    next.width = info.width;
    next.height = info.height;
    next.resolution = info.resolution;
    next.origin_x = origin.position.x;
    next.origin_y = origin.position.y;
    next.origin_yaw = yaw_of(origin.orientation);
    next.occupancy.assign(msg.data.begin(), msg.data.end());
    map_ = std::move(next);
    detector_->reset();
    status_ = "waiting_cloud";
  }

  void on_pose(const Odometry & msg)
  {
    const double stamp = stamp_seconds(msg.header.stamp);
    const auto & p = msg.pose.pose.position;
    const auto q = quaternion_of(msg.pose.pose.orientation);
    const auto & cov = msg.pose.covariance;
    const double xy_largest = (cov[0] + cov[7] +
      std::sqrt((cov[0] - cov[7]) * (cov[0] - cov[7]) + 4.0 * cov[1] * cov[1])) / 2.0;
    if (msg.header.frame_id != world_frame_ || msg.child_frame_id != base_frame_ ||
      !std::isfinite(stamp) || stamp <= 0 || !std::isfinite(p.x) ||
      !std::isfinite(p.y) || !std::isfinite(p.z) || !valid_quaternion(q) ||
      !std::isfinite(cov[0]) || !std::isfinite(cov[1]) || !std::isfinite(cov[7]) ||
      !std::isfinite(cov[35]) || cov[0] < 0 || cov[7] < 0 || cov[35] < 0 ||
      xy_largest > max_position_std_ * max_position_std_ ||
      cov[35] > max_yaw_std_ * max_yaw_std_)
    {
      ++invalid_poses_;
      return;
    }
    if (!poses_.empty() && stamp + 0.5 < poses_.back().stamp) {
      poses_.clear();
      pending_cloud_.reset();
      detector_->reset();
      last_successful_stamp_ = 0.0;
    }
    StampedPose pose;
    pose.stamp = stamp;
    pose.position = tf2::Vector3(p.x, p.y, p.z);
    pose.orientation = q.normalized();
    pose.var_x = cov[0];
    pose.var_y = cov[7];
    pose.var_yaw = cov[35];
    const auto it = std::lower_bound(poses_.begin(), poses_.end(), stamp,
      [](const StampedPose & item, double time) {return item.stamp < time;});
    if (it != poses_.end() && std::abs(it->stamp - stamp) < 1e-6) {
      *it = pose;
    } else {
      poses_.insert(it, pose);
    }
    while (poses_.size() > 300 ||
      (poses_.size() > 1 && poses_.back().stamp - poses_.front().stamp > pose_history_s_))
    {
      poses_.pop_front();
    }
  }

  std::optional<StampedPose> pose_at(double stamp, bool allow_nearest = true) const
  {
    if (poses_.empty()) {return std::nullopt;}
    auto after = std::lower_bound(poses_.begin(), poses_.end(), stamp,
      [](const StampedPose & item, double time) {return item.stamp < time;});
    if (after != poses_.end() && std::abs(after->stamp - stamp) < 1e-6) {
      return *after;
    }
    if (after != poses_.begin() && after != poses_.end()) {
      const auto & before = *(after - 1);
      const double gap = after->stamp - before.stamp;
      if (gap > 0 && gap <= max_pose_gap_s_) {
        const double part = (stamp - before.stamp) / gap;
        StampedPose out;
        out.stamp = stamp;
        out.bracket_gap_s = gap;
        out.selection = "interpolated";
        out.position = before.position.lerp(after->position, part);
        out.orientation = before.orientation.slerp(after->orientation, part).normalized();
        // Linear covariance interpolation can appear overconfident if the two
        // estimates share the same map correction. Keep the larger endpoint.
        out.var_x = std::max(before.var_x, after->var_x);
        out.var_y = std::max(before.var_y, after->var_y);
        out.var_yaw = std::max(before.var_yaw, after->var_yaw);
        return out;
      }
    }
    const StampedPose * nearest = nullptr;
    if (after != poses_.end()) {nearest = &*after;}
    if (after != poses_.begin()) {
      const auto & before = *(after - 1);
      if (!nearest || std::abs(before.stamp - stamp) < std::abs(nearest->stamp - stamp)) {
        nearest = &before;
      }
    }
    if (allow_nearest && nearest &&
      std::abs(nearest->stamp - stamp) <= pose_nearest_tolerance_s_)
    {
      StampedPose out = *nearest;
      out.bracket_gap_s = std::abs(nearest->stamp - stamp);
      out.selection = "nearest";
      return out;
    }
    return std::nullopt;
  }

  void on_cloud(PointCloud2::SharedPtr msg)
  {
    if (pending_cloud_) {++replaced_clouds_;}
    pending_cloud_ = std::move(msg);
    last_cloud_stamp_ = stamp_seconds(pending_cloud_->header.stamp);
  }

  std::optional<tf2::Transform> sensor_transform(
    const std::string & frame, const builtin_interfaces::msg::Time & stamp)
  {
    if (frame == base_frame_) {return tf2::Transform::getIdentity();}
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, frame, rclcpp::Time(stamp));
      const auto q = quaternion_of(tf.transform.rotation);
      const auto & p = tf.transform.translation;
      if (!valid_quaternion(q) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.z))
      {
        return std::nullopt;
      }
      return tf2::Transform(q.normalized(), tf2::Vector3(p.x, p.y, p.z));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void tick()
  {
    const double now_s = now().seconds();
    if (visible_state_ && now_s - last_visible_s_ > visible_timeout_s_) {
      publish_visible(false);
      clear_markers();
    }
    if (!pending_cloud_) {
      if (now_s - last_cloud_stamp_ > max_cloud_age_s_) {status_ = "waiting_cloud";}
      publish_diagnostics_if_due(now_s);
      return;
    }
    const auto & cloud = *pending_cloud_;
    const double stamp = stamp_seconds(cloud.header.stamp);
    const double age = now_s - stamp;
    if (!std::isfinite(stamp) || stamp <= 0 || age > max_cloud_age_s_ || age < -max_future_s_) {
      status_ = "stale_or_future_cloud";
      ++discarded_clouds_;
      pending_cloud_.reset();
      publish_visible(false);
      clear_markers();
      publish_diagnostics_if_due(now_s);
      return;
    }
    if (!map_ && !allow_mapless_) {status_ = "waiting_map"; publish_diagnostics_if_due(now_s); return;}
    const auto pose = pose_at(stamp);
    if (!pose) {status_ = "waiting_stamped_pose"; publish_diagnostics_if_due(now_s); return;}
    const auto base_from_sensor = sensor_transform(cloud.header.frame_id, cloud.header.stamp);
    if (!base_from_sensor) {status_ = "waiting_stamped_sensor_tf"; publish_diagnostics_if_due(now_s); return;}
    ParsedCloud parsed;
    const std::string parse_error = parse_cloud(cloud, max_cloud_points_, false, parsed);
    if (!parse_error.empty()) {
      status_ = parse_error;
      ++discarded_clouds_;
      pending_cloud_.reset();
      publish_visible(false);
      clear_markers();
      publish_diagnostics_if_due(now_s);
      return;
    }
    if (last_successful_stamp_ > 0 && stamp + 0.1 < last_successful_stamp_) {
      detector_->reset();
    }
    const tf2::Transform map_from_base(pose->orientation, pose->position);
    const tf2::Transform map_from_sensor = map_from_base * *base_from_sensor;
    const auto origin = map_from_sensor.getOrigin();
    const auto rotation = map_from_sensor.getRotation();
    core::Frame frame;
    frame.stamp = stamp;
    frame.points = std::move(parsed.points);
    frame.map_from_sensor.translation = {origin.x(), origin.y(), origin.z()};
    frame.map_from_sensor.qx = rotation.x();
    frame.map_from_sensor.qy = rotation.y();
    frame.map_from_sensor.qz = rotation.z();
    frame.map_from_sensor.qw = rotation.w();
    frame.pose_var_x = pose->var_x;
    frame.pose_var_y = pose->var_y;
    frame.pose_var_yaw = pose->var_yaw;
    frame.map = map_ ? &*map_ : nullptr;
    const auto result = detector_->process(frame);
    const auto cloud_stamp = cloud.header.stamp;
    pending_cloud_.reset();
    const bool processed = result.status == "measured_robot" ||
      result.status == "no_foreground_objects" ||
      result.status == "no_confirmed_robot" ||
      result.status == "map_alignment_inconsistent" ||
      result.status == "no_points_in_search_volume";
    if (processed) {last_successful_stamp_ = stamp;}
    last_packet_span_s_ = parsed.packet_span_s;
    last_selected_pose_stamp_s_ = pose->stamp;
    last_pose_gap_s_ = pose->bracket_gap_s;
    last_pose_selection_ = pose->selection;
    last_invalid_points_ = parsed.invalid_points;
    ++processed_clouds_;
    status_ = result.status;
    if (result.has_measurement) {
      odom_pub_->publish(make_odometry(cloud_stamp, result.measurement));
      last_visible_s_ = now_s;
    }
    publish_visible(result.has_measurement);
    publish_markers(cloud_stamp, result);
    publish_diagnostics(now_s, &result);
  }

  Odometry make_odometry(const builtin_interfaces::msg::Time & stamp, const core::Track & track) const
  {
    Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_;
    msg.child_frame_id = opponent_frame_;
    msg.pose.pose.position.x = track.center_map.x;
    msg.pose.pose.position.y = track.center_map.y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;  // tracking-frame axes align with map
    msg.pose.covariance.fill(0.0);
    msg.pose.covariance[0] = std::max(1e-5, track.position_covariance[0]);
    msg.pose.covariance[1] = track.position_covariance[1];
    msg.pose.covariance[6] = track.position_covariance[2];
    msg.pose.covariance[7] = std::max(1e-5, track.position_covariance[3]);
    msg.pose.covariance[14] = 0.05 * 0.05;
    msg.pose.covariance[21] = 1e4;
    msg.pose.covariance[28] = 1e4;
    msg.pose.covariance[35] = 1e4;  // robot body heading is unobserved
    msg.twist.covariance.fill(0.0);
    const double speed_variance = track.velocity_valid ?
      std::max(1e-4, track.velocity_variance) : 1e4;
    if (track.velocity_valid) {
      msg.twist.twist.linear.x = track.velocity_map.x;
      msg.twist.twist.linear.y = track.velocity_map.y;
    }
    msg.twist.covariance[0] = speed_variance;
    msg.twist.covariance[7] = speed_variance;
    msg.twist.covariance[14] = 1e4;
    msg.twist.covariance[21] = 1e4;
    msg.twist.covariance[28] = 1e4;
    msg.twist.covariance[35] = 1e4;
    return msg;
  }

  void publish_visible(bool visible)
  {
    std_msgs::msg::Bool msg;
    msg.data = visible;
    visible_pub_->publish(msg);
    visible_state_ = visible;
  }

  Marker marker(const builtin_interfaces::msg::Time & stamp,
    const std::string & ns, int id, int type) const
  {
    Marker out;
    out.header.frame_id = world_frame_;
    out.header.stamp = stamp;
    out.ns = ns;
    out.id = id;
    out.action = Marker::ADD;
    out.type = type;
    out.pose.orientation.w = 1.0;
    out.lifetime = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(0.7));
    return out;
  }

  void add_marker(MarkerArray & batch, Marker out, std::set<std::pair<std::string, int>> & keys)
  {
    keys.insert({out.ns, out.id});
    batch.markers.push_back(std::move(out));
  }

  void finish_markers(MarkerArray & batch, const builtin_interfaces::msg::Time & stamp,
    const std::set<std::pair<std::string, int>> & keys)
  {
    for (const auto & key : marker_keys_) {
      if (keys.find(key) != keys.end()) {continue;}
      auto removed = marker(stamp, key.first, key.second, Marker::SPHERE);
      removed.action = Marker::DELETE;
      batch.markers.push_back(std::move(removed));
    }
    marker_keys_ = keys;
    if (!batch.markers.empty()) {markers_pub_->publish(batch);}
  }

  void clear_markers()
  {
    if (marker_keys_.empty()) {return;}
    MarkerArray batch;
    finish_markers(batch, static_cast<builtin_interfaces::msg::Time>(now()), {});
  }

  void add_ellipse(MarkerArray & batch, std::set<std::pair<std::string, int>> & keys,
    const builtin_interfaces::msg::Time & stamp, const core::Track & track,
    const std::string & ns, int id, float r, float g, float b)
  {
    const auto & cov = track.position_covariance;
    const double a = std::max(1e-5, cov[0]);
    const double d = std::max(1e-5, cov[3]);
    const double off = 0.5 * (cov[1] + cov[2]);
    const double split = std::sqrt((a - d) * (a - d) + 4.0 * off * off);
    const double major = std::min(1.5, 2.0 * std::sqrt(std::max(0.0, (a + d + split) / 2.0)));
    const double minor = std::min(1.5, 2.0 * std::sqrt(std::max(0.0, (a + d - split) / 2.0)));
    const double angle = 0.5 * std::atan2(2.0 * off, a - d);
    auto out = marker(stamp, ns, id, Marker::LINE_STRIP);
    out.scale.x = 0.025;
    out.color.r = r; out.color.g = g; out.color.b = b; out.color.a = 0.85F;
    for (int i = 0; i <= 24; ++i) {
      const double theta = i * (2.0 * M_PI / 24.0);
      geometry_msgs::msg::Point p;
      p.x = track.center_map.x + std::cos(angle) * major * std::cos(theta) -
        std::sin(angle) * minor * std::sin(theta);
      p.y = track.center_map.y + std::sin(angle) * major * std::cos(theta) +
        std::cos(angle) * minor * std::sin(theta);
      p.z = 0.055;
      out.points.push_back(p);
    }
    add_marker(batch, std::move(out), keys);
  }

  void add_sample_points(MarkerArray & batch,
    std::set<std::pair<std::string, int>> & keys,
    const builtin_interfaces::msg::Time & stamp, const std::string & ns, int id,
    const std::vector<core::Vec3> & points, std::size_t & remaining,
    float r, float g, float b)
  {
    if (remaining == 0 || points.empty()) {return;}
    auto out = marker(stamp, ns, id, Marker::POINTS);
    out.scale.x = 0.035; out.scale.y = 0.035;
    out.color.r = r; out.color.g = g; out.color.b = b; out.color.a = 0.95F;
    const std::size_t count = std::min({remaining, points.size(), std::size_t{8}});
    for (std::size_t j = 0; j < count; ++j) {
      geometry_msgs::msg::Point p;
      p.x = points[j].x;
      p.y = points[j].y;
      p.z = points[j].z;
      out.points.push_back(p);
    }
    remaining -= count;
    add_marker(batch, std::move(out), keys);
  }

  void publish_markers(const builtin_interfaces::msg::Time & stamp, const core::Result & result)
  {
    MarkerArray batch;
    std::set<std::pair<std::string, int>> keys;
    std::size_t remaining_points = max_marker_points_;
    const auto candidate_count = std::min(max_marker_candidates_, result.candidates.size());
    for (std::size_t i = 0; i < candidate_count; ++i) {
      const auto & candidate = result.candidates[i];
      const int id = static_cast<int>(candidate.id % 2000000000ULL);
      float r = 0.55F, g = 0.55F, b = 0.55F;
      if (candidate.decision == core::Decision::accepted) {r = 0.10F; g = 0.95F; b = 0.30F;}
      if (candidate.decision == core::Decision::ambiguous) {r = 1.0F; g = 0.75F; b = 0.05F;}
      auto center = marker(stamp, "candidate", id, Marker::SPHERE);
      center.pose.position.x = candidate.center_map.x;
      center.pose.position.y = candidate.center_map.y;
      center.pose.position.z = 0.16;
      center.scale.x = std::clamp(candidate.width, 0.08, 0.7);
      center.scale.y = std::clamp(candidate.depth, 0.08, 0.7);
      center.scale.z = 0.08;
      center.color.r = r; center.color.g = g; center.color.b = b; center.color.a = 0.55F;
      add_marker(batch, std::move(center), keys);
      auto label = marker(stamp, "candidate_reason", id, Marker::TEXT_VIEW_FACING);
      label.pose.position.x = candidate.center_map.x;
      label.pose.position.y = candidate.center_map.y;
      label.pose.position.z = 0.53;
      label.scale.z = 0.10;
      label.color.r = r; label.color.g = g; label.color.b = b; label.color.a = 1.0F;
      std::ostringstream description;
      description << candidate.reason << " score=" << std::fixed << std::setprecision(2) <<
        candidate.robot_probability << " support=" << candidate.support_count <<
        " conflict=" << candidate.contradictory_count;
      label.text = description.str();
      add_marker(batch, std::move(label), keys);
      add_sample_points(batch, keys, stamp, "candidate_object", id,
        candidate.object_map, remaining_points, 0.35F, 0.45F, 0.55F);
      add_sample_points(batch, keys, stamp, "candidate_support", id,
        candidate.support_map, remaining_points, r, g, b);
      add_sample_points(batch, keys, stamp, "candidate_conflict", id,
        candidate.contradictory_map, remaining_points, 1.0F, 0.1F, 0.1F);
    }
    const auto track_count = std::min(max_marker_tracks_, result.tracks.size());
    for (std::size_t i = 0; i < track_count; ++i) {
      const auto & track = result.tracks[i];
      const int id = static_cast<int>(track.id % 2000000000ULL);
      auto center = marker(stamp, "track", id, Marker::SPHERE);
      center.pose.position.x = track.center_map.x;
      center.pose.position.y = track.center_map.y;
      center.pose.position.z = 0.25;
      center.scale.x = 0.09; center.scale.y = 0.09; center.scale.z = 0.09;
      center.color.r = track.confirmed ? 0.9F : 0.95F;
      center.color.g = track.confirmed ? 0.1F : 0.65F;
      center.color.b = 0.05F;
      center.color.a = 1.0F;
      add_marker(batch, std::move(center), keys);
      auto label = marker(stamp, "track_age", id, Marker::TEXT_VIEW_FACING);
      label.pose.position.x = track.center_map.x;
      label.pose.position.y = track.center_map.y;
      label.pose.position.z = 0.68;
      label.scale.z = 0.10;
      label.color.r = 1.0F; label.color.g = 0.7F; label.color.b = 0.3F; label.color.a = 1.0F;
      std::ostringstream description;
      description << "track=" << track.id << " score=" << std::fixed << std::setprecision(2) <<
        track.probability << " age=" << track.age_since_measurement <<
        " hits=" << track.measured_hits;
      label.text = description.str();
      add_marker(batch, std::move(label), keys);
    }
    if (result.has_measurement) {
      const auto & measured = result.measurement;
      auto target = marker(stamp, "selected_measurement", 1, Marker::CYLINDER);
      target.pose.position.x = measured.center_map.x;
      target.pose.position.y = measured.center_map.y;
      target.pose.position.z = 0.18;
      target.scale.x = 0.356; target.scale.y = 0.356; target.scale.z = 0.36;
      target.color.r = 1.0F; target.color.g = 0.10F; target.color.b = 0.10F; target.color.a = 0.40F;
      add_marker(batch, std::move(target), keys);
      add_ellipse(batch, keys, stamp, measured, "measurement_uncertainty", 1, 1.0F, 0.2F, 0.2F);
    }
    if (result.has_prediction) {
      const auto & predicted = result.prediction;
      auto center = marker(stamp, "prediction_only", 1, Marker::SPHERE);
      center.pose.position.x = predicted.center_map.x;
      center.pose.position.y = predicted.center_map.y;
      center.pose.position.z = 0.12;
      center.scale.x = 0.20; center.scale.y = 0.20; center.scale.z = 0.06;
      center.color.r = 0.15F; center.color.g = 0.55F; center.color.b = 1.0F; center.color.a = 0.65F;
      add_marker(batch, std::move(center), keys);
      add_ellipse(batch, keys, stamp, predicted, "prediction_uncertainty", 1, 0.2F, 0.6F, 1.0F);
    }
    finish_markers(batch, stamp, keys);
  }

  void publish_diagnostics_if_due(double now_s)
  {
    if (now_s - last_diagnostics_s_ >= 0.2) {publish_diagnostics(now_s, nullptr);}
  }

  void publish_diagnostics(double now_s, const core::Result * result)
  {
    last_diagnostics_s_ = now_s;
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::setprecision(17) << "{\"stamp_s\":" << last_successful_stamp_ <<
      ",\"cloud_stamp_s\":" << last_cloud_stamp_ <<
      ",\"pose_stamp_s\":" << last_selected_pose_stamp_s_ <<
      ",\"latest_pose_stamp_s\":" << (poses_.empty() ? 0.0 : poses_.back().stamp) <<
      ",\"pose_selection\":" << json_quote(last_pose_selection_) <<
      ",\"pose_gap_s\":" << last_pose_gap_s_ <<
      ",\"status\":" << json_quote(status_) <<
      ",\"backend\":\"jr_perception_cpp\"" <<
      ",\"costmap_source\":\"static\"" <<
      ",\"map_ready\":" << (map_.has_value() ? "true" : "false") <<
      ",\"mapless_replay\":" << (allow_mapless_ ? "true" : "false") <<
      ",\"visible\":" << (visible_state_ ? "true" : "false") <<
      ",\"processed_clouds\":" << processed_clouds_ <<
      ",\"replaced_clouds\":" << replaced_clouds_ <<
      ",\"discarded_clouds\":" << discarded_clouds_ <<
      ",\"invalid_poses\":" << invalid_poses_ <<
      ",\"invalid_points\":" << last_invalid_points_ <<
      ",\"packet_span_s\":" << last_packet_span_s_ <<
      ",\"cloud_age_s\":" << (last_cloud_stamp_ > 0.0 ? now_s - last_cloud_stamp_ : 0.0) <<
      ",\"score_semantics\":\"uncalibrated_heuristic\"";
    if (result) {
      out << ",\"candidate_count\":" << result->candidates.size() <<
        ",\"map_alignment_share\":" << result->map_alignment_share <<
        ",\"track_count\":" << result->tracks.size() <<
        ",\"measurement\":" << (result->has_measurement ? "true" : "false") <<
        ",\"prediction\":" << (result->has_prediction ? "true" : "false") <<
        ",\"transform_ms\":" << result->timing.transform_ms <<
        ",\"segmentation_ms\":" << result->timing.segmentation_ms <<
        ",\"scoring_ms\":" << result->timing.scoring_ms <<
        ",\"tracking_ms\":" << result->timing.tracking_ms <<
        ",\"total_ms\":" << result->timing.total_ms <<
        ",\"candidate_diagnostics_truncated\":" <<
        (result->candidates.size() > max_marker_candidates_ ? "true" : "false") <<
        ",\"candidates\":[";
      for (std::size_t i = 0; i < std::min(max_marker_candidates_, result->candidates.size()); ++i) {
        const auto & candidate = result->candidates[i];
        if (i) {out << ',';}
        const char * decision = candidate.decision == core::Decision::accepted ? "accepted" :
          candidate.decision == core::Decision::ambiguous ? "ambiguous" : "rejected";
        out << "{\"id\":" << candidate.id <<
          ",\"track_id\":" << candidate.track_id <<
          ",\"decision\":" << json_quote(decision) <<
          ",\"reason\":" << json_quote(candidate.reason) <<
          ",\"score\":" << candidate.robot_probability <<
          ",\"x\":" << candidate.center_map.x <<
          ",\"y\":" << candidate.center_map.y <<
          ",\"points\":" << candidate.point_count <<
          ",\"support\":" << candidate.support_count <<
          ",\"contradictory\":" << candidate.contradictory_count <<
          ",\"complete_body_viable\":" << (candidate.complete_body_viable ? "true" : "false") <<
          ",\"temporal_change\":" << (candidate.temporal_change ? "true" : "false") <<
          ",\"middle_to_upper_ratio\":" << candidate.middle_to_upper_ratio << '}';
      }
      out << "]";
      if (result->has_prediction) {
        out << ",\"prediction_age_s\":" << result->prediction.age_since_measurement;
      }
    }
    out << '}';
    msg.data = out.str();
    diagnostics_pub_->publish(msg);
  }

  std::string cloud_topic_, pose_topic_, map_topic_, world_frame_, base_frame_, opponent_frame_;
  bool allow_mapless_ = false;
  double max_cloud_age_s_, max_future_s_, max_pose_gap_s_, pose_nearest_tolerance_s_;
  double pose_history_s_, max_position_std_, max_yaw_std_, visible_timeout_s_;
  std::size_t max_cloud_points_, max_marker_candidates_, max_marker_tracks_, max_marker_points_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<core::Detector> detector_;
  std::optional<core::Map> map_;
  std::deque<StampedPose> poses_;
  PointCloud2::SharedPtr pending_cloud_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::set<std::pair<std::string, int>> marker_keys_;
  std::string status_ = "waiting_map";
  double last_cloud_stamp_ = 0.0;
  double last_successful_stamp_ = 0.0;
  double last_visible_s_ = 0.0;
  double last_diagnostics_s_ = 0.0;
  double last_packet_span_s_ = 0.0;
  double last_selected_pose_stamp_s_ = 0.0;
  double last_pose_gap_s_ = 0.0;
  std::string last_pose_selection_ = "none";
  std::size_t last_invalid_points_ = 0;
  std::uint64_t processed_clouds_ = 0;
  std::uint64_t replaced_clouds_ = 0;
  std::uint64_t discarded_clouds_ = 0;
  std::uint64_t invalid_poses_ = 0;
  bool visible_state_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OpponentNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "opponent_detector_cpp: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
