#include "robot_body_filter/robot_body_filter.hpp"

#include <cstring>
#include <limits>


namespace robot_body_filter {

namespace {

int fieldOffset(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
    for (const auto& field : cloud.fields) {
        if (field.name == name) {
            return static_cast<int>(field.offset);
        }
    }
    return -1;
}

}  // namespace


void RobotBodyFilter::setStaticBoxes(const std::vector<CropBox>& boxes) {
    // Позы фреймов ещё не известны — боксы встанут на место в updateStaticBoxes().
    static_boxes_ = boxes;
}


void RobotBodyFilter::setDynamicBoxes(const std::vector<CropBox>& boxes) {
    dynamic_boxes_ = boxes;
}


bool RobotBodyFilter::updateStaticBoxes(
    const std::vector<Eigen::Isometry3d>& frame_poses
) {
    if (frame_poses.size() != static_boxes_.size()) {
        return false;
    }

    for (size_t i = 0; i < static_boxes_.size(); ++i) {
        static_boxes_[i].applyFramePose(frame_poses[i], false);
    }

    updateGlobalAabb();
    return true;
}


void RobotBodyFilter::updateGlobalAabb() {
    global_aabb_min_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    global_aabb_max_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    has_boxes_ = false;

    for (const auto& box : static_boxes_) {
        global_aabb_min_ = global_aabb_min_.cwiseMin(box.aabb_min);
        global_aabb_max_ = global_aabb_max_.cwiseMax(box.aabb_max);
        has_boxes_ = true;
    }

    for (const auto& box : dynamic_boxes_) {
        global_aabb_min_ = global_aabb_min_.cwiseMin(box.swept_aabb_min);
        global_aabb_max_ = global_aabb_max_.cwiseMax(box.swept_aabb_max);
        has_boxes_ = true;
    }
}


bool RobotBodyFilter::isPointInsideAnyBox(const Eigen::Vector3d& point) const {
    for (const auto& box : static_boxes_) {
        if (box.contains(point)) {
            return true;
        }
    }

    for (const auto& box : dynamic_boxes_) {
        if (box.contains(point)) {
            return true;
        }
    }

    return false;
}


bool RobotBodyFilter::updateDynamicBoxes(
    const std::vector<Eigen::Isometry3d>& joint_poses
) {
    if (joint_poses.size() != dynamic_boxes_.size()) {
        return false;
    }

    for (size_t i = 0; i < dynamic_boxes_.size(); ++i) {
        dynamic_boxes_[i].applyFramePose(joint_poses[i], true);
    }

    updateGlobalAabb();
    return true;
}


FilterStats RobotBodyFilter::filter(
    const sensor_msgs::msg::PointCloud2& cloud_in,
    sensor_msgs::msg::PointCloud2& cloud_out
) const {
    const int x_offset = fieldOffset(cloud_in, "x");
    const int y_offset = fieldOffset(cloud_in, "y");
    const int z_offset = fieldOffset(cloud_in, "z");

    const size_t points_in = (cloud_in.point_step > 0)
        ? cloud_in.data.size() / cloud_in.point_step
        : 0;

    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || points_in == 0) {
        cloud_out = cloud_in;
        return FilterStats(points_in, 0, points_in);
    }

    cloud_out.header = cloud_in.header;
    cloud_out.fields = cloud_in.fields;
    cloud_out.is_bigendian = cloud_in.is_bigendian;
    cloud_out.point_step = cloud_in.point_step;
    cloud_out.data.resize(cloud_in.data.size());

    size_t output_size = 0;
    size_t points_removed = 0;

    for (size_t offset = 0; offset + cloud_in.point_step <= cloud_in.data.size(); offset += cloud_in.point_step) {

        float x;
        float y;
        float z;
        std::memcpy(&x, &cloud_in.data[offset + x_offset], sizeof(float));
        std::memcpy(&y, &cloud_in.data[offset + y_offset], sizeof(float));
        std::memcpy(&z, &cloud_in.data[offset + z_offset], sizeof(float));

        const Eigen::Vector3d point(x, y, z);

        const bool near_robot = has_boxes_
            && point.x() >= global_aabb_min_.x() && point.x() <= global_aabb_max_.x()
            && point.y() >= global_aabb_min_.y() && point.y() <= global_aabb_max_.y()
            && point.z() >= global_aabb_min_.z() && point.z() <= global_aabb_max_.z();

        if (near_robot && isPointInsideAnyBox(point)) {
            ++points_removed;
            continue;
        }

        std::memcpy(
            &cloud_out.data[output_size],
            &cloud_in.data[offset],
            cloud_in.point_step
        );
        output_size += cloud_in.point_step;
    }

    cloud_out.data.resize(output_size);

    const size_t points_out = output_size / cloud_in.point_step;

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(points_out);
    cloud_out.row_step = static_cast<uint32_t>(output_size);
    cloud_out.is_dense = cloud_in.is_dense;

    return FilterStats(points_in, points_removed, points_out);
}


void RobotBodyFilter::computeMask(
    const sensor_msgs::msg::PointCloud2& cloud_in,
    std::vector<uint8_t>& mask
) const {
    const int x_offset = fieldOffset(cloud_in, "x");
    const int y_offset = fieldOffset(cloud_in, "y");
    const int z_offset = fieldOffset(cloud_in, "z");

    const size_t points_in = (cloud_in.point_step > 0)
        ? cloud_in.data.size() / cloud_in.point_step
        : 0;

    mask.assign(points_in, 0);

    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
        return;
    }

    size_t index = 0;
    for (size_t offset = 0; offset + cloud_in.point_step <= cloud_in.data.size();
         offset += cloud_in.point_step, ++index) {

        float x;
        float y;
        float z;
        std::memcpy(&x, &cloud_in.data[offset + x_offset], sizeof(float));
        std::memcpy(&y, &cloud_in.data[offset + y_offset], sizeof(float));
        std::memcpy(&z, &cloud_in.data[offset + z_offset], sizeof(float));

        const Eigen::Vector3d point(x, y, z);
        mask[index] = isPointInsideAnyBox(point) ? 1 : 0;
    }
}


std::vector<std::string> RobotBodyFilter::staticFrames() const {
    std::vector<std::string> frames;
    frames.reserve(static_boxes_.size());
    for (const auto& box : static_boxes_) {
        frames.push_back(box.frame);
    }
    return frames;
}


std::vector<std::string> RobotBodyFilter::jointFrames() const {
    std::vector<std::string> frames;
    frames.reserve(dynamic_boxes_.size());
    for (const auto& box : dynamic_boxes_) {
        frames.push_back(box.frame);
    }
    return frames;
}

}  // namespace robot_body_filter
