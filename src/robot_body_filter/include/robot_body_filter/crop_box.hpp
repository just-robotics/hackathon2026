#ifndef ROBOT_BODY_FILTER__CROP_BOX_HPP
#define ROBOT_BODY_FILTER__CROP_BOX_HPP


#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace robot_body_filter {

struct CropBox {
    std::string name;

    /// Фрейм звена, в СК которого заданы min/max. Обязателен для любого бокса.
    /// Поза статического бокса читается из TF один раз при старте,
    /// динамического — на каждом облаке.
    std::string frame;

    /// Поза frame в base_frame. Заполняется из TF, в конфиге не задаётся.
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();

    Eigen::Vector3d padding = Eigen::Vector3d::Zero();

    bool has_angle_range = false;
    double angle_min = 0.0;
    double angle_max = 0.0;

    bool axis_aligned = true;

    Eigen::Isometry3d world_to_box = Eigen::Isometry3d::Identity();

    Eigen::Vector3d aabb_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d aabb_max = Eigen::Vector3d::Zero();

    Eigen::Vector3d swept_aabb_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d swept_aabb_max = Eigen::Vector3d::Zero();

    void updateTransform();

    void updateSweptAabb(bool dynamic);

    /// Ставит бокс по позе его фрейма и пересобирает производные величины.
    void applyFramePose(const Eigen::Isometry3d& frame_pose, bool dynamic);

    bool insideAabb(const Eigen::Vector3d& point) const {
        return point.x() >= aabb_min.x() && point.x() <= aabb_max.x()
            && point.y() >= aabb_min.y() && point.y() <= aabb_max.y()
            && point.z() >= aabb_min.z() && point.z() <= aabb_max.z();
    }

    bool contains(const Eigen::Vector3d& point) const;
};

}  // namespace robot_body_filter


#endif  // ROBOT_BODY_FILTER__CROP_BOX_HPP
