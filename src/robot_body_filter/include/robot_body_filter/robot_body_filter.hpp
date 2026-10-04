#ifndef ROBOT_BODY_FILTER__ROBOT_BODY_FILTER_HPP
#define ROBOT_BODY_FILTER__ROBOT_BODY_FILTER_HPP


#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "robot_body_filter/crop_box.hpp"


namespace robot_body_filter {

class FilterStats {
private:
    size_t points_in_ = 0;
    size_t points_removed_ = 0;
    size_t points_out_ = 0;

public:
    FilterStats() = default;

    FilterStats(size_t points_in, size_t points_removed, size_t points_out)
        : points_in_(points_in),
          points_removed_(points_removed),
          points_out_(points_out) {}

    size_t pointsIn() const { return points_in_; }
    size_t pointsRemoved() const { return points_removed_; }
    size_t pointsOut() const { return points_out_; }
};


class RobotBodyFilter {
private:
    std::vector<CropBox> static_boxes_;
    std::vector<CropBox> dynamic_boxes_;

    Eigen::Vector3d global_aabb_min_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d global_aabb_max_ = Eigen::Vector3d::Zero();

    bool has_boxes_ = false;

    void updateGlobalAabb();

    bool isPointInsideAnyBox(const Eigen::Vector3d& point) const;

public:
    RobotBodyFilter() = default;

    void setStaticBoxes(const std::vector<CropBox>& boxes);
    void setDynamicBoxes(const std::vector<CropBox>& boxes);

    /// Ставит статические боксы по позам их фреймов. Вызывается один раз при
    /// старте: звенья корпуса неподвижны относительно base_frame.
    bool updateStaticBoxes(const std::vector<Eigen::Isometry3d>& frame_poses);

    bool updateDynamicBoxes(const std::vector<Eigen::Isometry3d>& joint_poses);

    FilterStats filter(
        const sensor_msgs::msg::PointCloud2& cloud_in,
        sensor_msgs::msg::PointCloud2& cloud_out
    ) const;

    void computeMask(
        const sensor_msgs::msg::PointCloud2& cloud_in,
        std::vector<uint8_t>& mask
    ) const;

    const std::vector<CropBox>& staticBoxes() const { return static_boxes_; }
    const std::vector<CropBox>& dynamicBoxes() const { return dynamic_boxes_; }

    /// Фреймы статических боксов в порядке static_boxes_.
    std::vector<std::string> staticFrames() const;

    /// Фреймы динамических боксов в порядке dynamic_boxes_.
    std::vector<std::string> jointFrames() const;

    bool empty() const { return static_boxes_.empty() && dynamic_boxes_.empty(); }
};

}  // namespace robot_body_filter


#endif  // ROBOT_BODY_FILTER__ROBOT_BODY_FILTER_HPP
