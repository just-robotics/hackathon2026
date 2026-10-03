// SPDX-License-Identifier: Apache-2.0
// Keep Nav2 marking/raytracing; remove only cells explicitly reclassified by
// the sensor-only small-box filter, inside this dynamic layer, never StaticLayer.
#include <cmath>
#include <mutex>
#include <string>
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace hsl_nav2_control
{
class SemanticObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override
  {
    nav2_costmap_2d::ObstacleLayer::onInitialize();
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("costmap node expired");}
    declareParameter("ignored_topic", rclcpp::ParameterValue("navigation/ignored_obstacles"));
    std::string topic;
    node->get_parameter(name_ + ".ignored_topic", topic);
    subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(mutex_); ignored_ = msg;
      });
  }

  void updateBounds(double x, double y, double yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    nav2_costmap_2d::ObstacleLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y);
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    {std::lock_guard<std::mutex> guard(mutex_); msg = ignored_;}
    if (!msg || msg->header.frame_id != layered_costmap_->getGlobalFrameID()) {return;}
    const double age = (clock_->now() - rclcpp::Time(msg->header.stamp)).seconds();
    if (age < 0 || age > 0.8) {return;}
    try {
      sensor_msgs::PointCloud2ConstIterator<float> px(*msg, "x"), py(*msg, "y");
      for (; px != px.end(); ++px, ++py) {
        if (!std::isfinite(*px) || !std::isfinite(*py)) {continue;}
        unsigned int mx, my;
        if (!worldToMap(*px, *py, mx, my)) {continue;}
        // Match the global memory's one-cell uncertainty in a reclassified hit.
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int cx = static_cast<int>(mx) + dx, cy = static_cast<int>(my) + dy;
            if (cx < 0 || cy < 0 || cx >= static_cast<int>(getSizeInCellsX()) ||
              cy >= static_cast<int>(getSizeInCellsY())) {continue;}
            setCost(cx, cy, nav2_costmap_2d::FREE_SPACE);
            double wx, wy; mapToWorld(cx, cy, wx, wy);
            touch(wx - getResolution(), wy - getResolution(), min_x, min_y, max_x, max_y);
            touch(wx + getResolution(), wy + getResolution(), min_x, min_y, max_x, max_y);
          }
        }
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Invalid ignored cloud: %s", e.what());
    }
  }
private:
  std::mutex mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr ignored_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace hsl_nav2_control
PLUGINLIB_EXPORT_CLASS(hsl_nav2_control::SemanticObstacleLayer, nav2_costmap_2d::Layer)
