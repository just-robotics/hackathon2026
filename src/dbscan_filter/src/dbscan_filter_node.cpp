#include "dbscan_filter/dbscan_filter_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>


namespace dbscan_filter {

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


DbscanFilterNode::DbscanFilterNode()
    : rclcpp::Node("dbscan_filter_node") {

    params_.eps = this->declare_parameter<double>("eps", 0.2);
    params_.min_points = this->declare_parameter<int>("min_points", 4);
    params_.min_cluster_size = this->declare_parameter<int>("min_cluster_size", 10);
    params_.min_cluster_size_radius =
        this->declare_parameter<double>("min_cluster_size_radius", 3.0);
    params_.min_range = this->declare_parameter<double>("min_range", 0.05);
    const auto max_points = this->declare_parameter<int>("max_points", 30000);
    max_points_ = max_points > 0 ? static_cast<size_t>(max_points) : 0;

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input/pointcloud",
        rclcpp::SensorDataQoS(),
        std::bind(&DbscanFilterNode::cloudCallback, this, std::placeholders::_1)
    );

    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/output/pointcloud",
        rclcpp::SensorDataQoS()
    );

    RCLCPP_INFO(
        this->get_logger(),
        "dbscan_filter: eps %.3f m, min_points %d, min_cluster_size %d "
        "в радиусе %.2f m, min_range %.3f m, max_points %zu",
        params_.eps,
        params_.min_points,
        params_.min_cluster_size,
        params_.min_cluster_size_radius,
        params_.min_range,
        max_points_
    );
}


bool DbscanFilterNode::extractPoints(const sensor_msgs::msg::PointCloud2& cloud) {
    const int x_offset = fieldOffset(cloud, "x");
    const int y_offset = fieldOffset(cloud, "y");
    const int z_offset = fieldOffset(cloud, "z");

    points_.clear();

    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || cloud.point_step == 0) {
        return false;
    }

    const size_t count = cloud.data.size() / cloud.point_step;
    points_.reserve(count);

    for (size_t offset = 0; offset + cloud.point_step <= cloud.data.size(); offset += cloud.point_step) {
        float x;
        float y;
        float z;
        std::memcpy(&x, &cloud.data[offset + x_offset], sizeof(float));
        std::memcpy(&y, &cloud.data[offset + y_offset], sizeof(float));
        std::memcpy(&z, &cloud.data[offset + z_offset], sizeof(float));

        points_.emplace_back(x, y, z);
    }

    return true;
}


void DbscanFilterNode::buildOutput(
    const sensor_msgs::msg::PointCloud2& cloud_in,
    sensor_msgs::msg::PointCloud2& cloud_out
) const {
    cloud_out.header = cloud_in.header;
    cloud_out.fields = cloud_in.fields;
    cloud_out.is_bigendian = cloud_in.is_bigendian;
    cloud_out.point_step = cloud_in.point_step;
    cloud_out.height = 1;
    cloud_out.is_dense = true;
    cloud_out.data.resize(cloud_in.data.size());

    size_t kept = 0;

    // Точка копируется целиком по point_step: кроме xyz у Livox есть
    // intensity, tag и line, и шумодаву незачем их терять.
    for (size_t index = 0; index < labels_.size(); ++index) {
        if (labels_[index] < 0) {
            continue;
        }

        std::memcpy(
            &cloud_out.data[kept * cloud_in.point_step],
            &cloud_in.data[index * cloud_in.point_step],
            cloud_in.point_step
        );
        ++kept;
    }

    cloud_out.data.resize(kept * cloud_in.point_step);
    cloud_out.width = static_cast<uint32_t>(kept);
    cloud_out.row_step = static_cast<uint32_t>(kept * cloud_in.point_step);
}


void DbscanFilterNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
) {
    if (!extractPoints(*msg)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "в облаке нет полей x/y/z, пропускаю кадр"
        );
        return;
    }

    if (points_.empty()) {
        filtered_cloud_pub_->publish(*msg);
        return;
    }

    if (max_points_ > 0 && points_.size() > max_points_) {
        // Кадр плотнее ожидаемого. Прорежать выборкой нельзя -- это меняет
        // локальную плотность, на которой держится весь критерий DBSCAN, и
        // шумом стали бы помечаться точки реальных препятствий. Честнее
        // пропустить кадр как есть и сказать об этом.
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "в кадре %zu точек при max_points %zu, облако отдано без фильтрации",
            points_.size(),
            max_points_
        );
        filtered_cloud_pub_->publish(*msg);
        return;
    }

    const size_t clusters = cluster(points_, params_, labels_);
    const size_t points_small = dropSmallClusters(points_, labels_, clusters, params_);

    sensor_msgs::msg::PointCloud2 cloud_out;
    buildOutput(*msg, cloud_out);

    filtered_cloud_pub_->publish(cloud_out);

    RCLCPP_DEBUG(
        this->get_logger(),
        "in %zu, clusters %zu, small %zu, out %u",
        points_.size(),
        clusters,
        points_small,
        cloud_out.width
    );
}

}  // namespace dbscan_filter


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dbscan_filter::DbscanFilterNode>());
    rclcpp::shutdown();

    return 0;
}
