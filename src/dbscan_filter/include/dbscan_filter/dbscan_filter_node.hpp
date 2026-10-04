#ifndef DBSCAN_FILTER__DBSCAN_FILTER_NODE_HPP
#define DBSCAN_FILTER__DBSCAN_FILTER_NODE_HPP


#include <memory>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dbscan_filter/dbscan.hpp"


namespace dbscan_filter {

/// Шумодав на DBSCAN: выбрасывает из облака точки, не попавшие ни в один
/// достаточно крупный кластер.
///
/// Ставится после robot_body_filter: корпус робота уже вырезан, и всё, что
/// осталось разреженным, -- это одиночные отражения, пыль и фантомные точки
/// на кромках. Геометрия кадра не меняется, поля сохраняются.
class DbscanFilterNode : public rclcpp::Node {
private:
    DbscanParams params_;

    /// Предохранитель от аномально плотных кадров: DBSCAN растёт быстрее
    /// линейного и на них не укладывается в период лидара. Такой кадр
    /// проходит без фильтрации. 0 -- без ограничения.
    size_t max_points_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;

    // Буферы переиспользуются между кадрами, чтобы не аллоцировать на каждом.
    std::vector<Eigen::Vector3f> points_;
    std::vector<int32_t> labels_;

public:
    DbscanFilterNode();

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    /// Достаёт xyz в points_. false -- в облаке нет нужных полей.
    bool extractPoints(const sensor_msgs::msg::PointCloud2& cloud);

    /// Собирает выход из точек, у которых labels_ >= 0.
    void buildOutput(
        const sensor_msgs::msg::PointCloud2& cloud_in,
        sensor_msgs::msg::PointCloud2& cloud_out
    ) const;
};

}  // namespace dbscan_filter


#endif  // DBSCAN_FILTER__DBSCAN_FILTER_NODE_HPP
