#ifndef ROBOT_BODY_FILTER__ROBOT_BODY_FILTER_NODE_HPP
#define ROBOT_BODY_FILTER__ROBOT_BODY_FILTER_NODE_HPP


#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_body_filter/robot_body_filter.hpp"


namespace robot_body_filter {

using PolygonStamped = geometry_msgs::msg::PolygonStamped;


class RobotBodyFilterNode : public rclcpp::Node {
private:
    std::string base_frame_;
    double tf_timeout_;
    double initial_tf_timeout_;

    RobotBodyFilter filter_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;

    // По паблишеру на бокс: у autoware нода обслуживает один бокс и публикует
    // ~/crop_box_polygon, у нас боксов несколько, поэтому имя бокса входит в топик.
    std::vector<rclcpp::Publisher<PolygonStamped>::SharedPtr> crop_box_polygon_pubs_;

public:
    RobotBodyFilterNode();

    std::vector<CropBox> declareBoxes(const std::string& prefix, bool dynamic);

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    bool lookupFramePoses(
        const std::vector<std::string>& frames,
        const rclcpp::Time& stamp,
        std::vector<Eigen::Isometry3d>& poses
    );

    bool waitForInitialPoses();

    bool transformCloudToBaseFrame(
        const sensor_msgs::msg::PointCloud2& cloud_in,
        sensor_msgs::msg::PointCloud2& cloud_out
    );

    void publishCropBoxPolygons();
};

}  // namespace robot_body_filter


#endif  // ROBOT_BODY_FILTER__ROBOT_BODY_FILTER_NODE_HPP
