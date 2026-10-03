#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <iomanip>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"

using Cloud = sensor_msgs::msg::PointCloud2;

// FLOAT32 decoding with no alignment assumption; Livox fields can be packed.
float read_float(const uint8_t * p, bool big_endian)
{
  uint32_t bits = 0;
  for (int i = 0; i < 4; ++i) {
    bits |= static_cast<uint32_t>(p[i]) << (8 * (big_endian ? 3 - i : i));
  }
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

class LidarFilter : public rclcpp::Node
{
public:
  LidarFilter() : Node("real_lidar_filter")
  {
    centers_ = declare_parameter<std::vector<double>>("self_occlusion_centers_deg",
      {15.0, 165.0, -165.0, -15.0});
    half_angle_ = declare_parameter("self_occlusion_half_angle_deg", 5.0);
    max_range_ = declare_parameter("self_occlusion_max_range", 0.45);
    quality_ = declare_parameter("filter_low_confidence", true);
    if (!std::isfinite(half_angle_) || half_angle_ <= 0 || half_angle_ >= 45 ||
      !std::isfinite(max_range_) || max_range_ < 0 || max_range_ > 1 ||
      !std::all_of(centers_.begin(), centers_.end(), [](double a){return std::isfinite(a);}))
    {
      throw std::invalid_argument("invalid LiDAR self-occlusion geometry");
    }
    auto topic = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    pub_ = create_publisher<Cloud>("/sensing/lidar/points_filtered", rclcpp::SensorDataQoS());
    diag_ = create_publisher<std_msgs::msg::String>("/sensing/lidar/filter_diagnostics", 10);
    sub_ = create_subscription<Cloud>(topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](const Cloud::ConstSharedPtr msg){process(*msg);});
  }

private:
  void process(const Cloud & input)
  {
    std::array<uint32_t, 3> xyz{};
    std::array<bool, 3> found{};
    uint32_t tag_offset = 0;
    bool tag_present = false;
    for (const auto & f : input.fields) {
      for (size_t i = 0; i < 3; ++i) {
        if (f.name == std::array<const char *, 3>{"x", "y", "z"}[i]) {
          if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 || f.count != 1 ||
            static_cast<uint64_t>(f.offset) + 4 > input.point_step) {invalid(); return;}
          xyz[i] = f.offset; found[i] = true;
        }
      }
      if (f.name == "tag") {
        if (f.datatype != sensor_msgs::msg::PointField::UINT8 || f.count != 1 ||
          f.offset >= input.point_step) {invalid(); return;}
        tag_present = true; tag_offset = f.offset;
      }
    }
    const uint64_t count = static_cast<uint64_t>(input.width) * input.height;
    const uint64_t row_bytes = static_cast<uint64_t>(input.width) * input.point_step;
    if (!std::all_of(found.begin(), found.end(), [](bool v){return v;}) ||
      input.point_step == 0 || row_bytes > input.row_step ||
      static_cast<uint64_t>(input.row_step) * input.height > input.data.size() ||
      count * input.point_step > std::numeric_limits<uint32_t>::max()) {invalid(); return;}
    Cloud output;
    output.header = input.header; output.fields = input.fields;
    output.height = 1; output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step; output.is_dense = true;
    output.data.reserve(static_cast<size_t>(count * input.point_step));
    uint64_t nonfinite = 0, low = 0, shadow = 0;
    for (uint32_t row = 0; row < input.height; ++row) {
      for (uint32_t col = 0; col < input.width; ++col) {
        const auto p = input.data.data() + static_cast<size_t>(row) * input.row_step +
          static_cast<size_t>(col) * input.point_step;
        const double x = read_float(p + xyz[0], input.is_bigendian);
        const double y = read_float(p + xyz[1], input.is_bigendian);
        const double z = read_float(p + xyz[2], input.is_bigendian);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {++nonfinite; continue;}
        const uint8_t tag = tag_present ? p[tag_offset] : 0;
        if (quality_ && ((tag & 3) >= 2 || ((tag >> 2) & 3) >= 2 || ((tag >> 4) & 3) >= 2)) {
          ++low; continue;
        }
        bool occluded = false;
        if (std::sqrt(x * x + y * y + z * z) <= max_range_) {
          const double azimuth = std::atan2(y, x) * 180.0 / std::acos(-1.0);
          for (double center : centers_) {
            double delta = std::fmod(azimuth - center + 180.0, 360.0);
            if (delta < 0) {delta += 360.0;}
            if (std::abs(delta - 180.0) <= half_angle_) {occluded = true; break;}
          }
        }
        if (occluded) {++shadow; continue;}
        output.data.insert(output.data.end(), p, p + input.point_step);
        ++output.width;
      }
    }
    output.row_step = output.width * output.point_step;
    pub_->publish(output);
    std::ostringstream text;
    text << "{\"input_points\":" << count << ",\"kept_points\":" << output.width <<
      ",\"nonfinite_points\":" << nonfinite << ",\"low_confidence_points\":" << low <<
      ",\"rod_shadow_points\":" << shadow << ",\"tag_present\":" << (tag_present ? "true" : "false") <<
      ",\"stamp_s\":" << std::fixed << std::setprecision(9) <<
      (input.header.stamp.sec + input.header.stamp.nanosec * 1e-9) << "}";
    std_msgs::msg::String diag; diag.data = text.str(); diag_->publish(diag);
  }
  void invalid()
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid LiDAR PointCloud2 layout");
  }
  std::vector<double> centers_;
  double half_angle_, max_range_;
  bool quality_;
  rclcpp::Publisher<Cloud>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_;
  rclcpp::Subscription<Cloud>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<LidarFilter>());}
  catch (const std::exception & e) {fprintf(stderr, "%s\n", e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
