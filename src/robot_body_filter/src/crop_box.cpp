#include "robot_body_filter/crop_box.hpp"

#include <cmath>
#include <limits>


namespace robot_body_filter {

namespace {

constexpr double kRotationEpsilon = 1e-9;

constexpr int kSweptSamples = 24;


void expand(
    Eigen::Vector3d& aabb_min,
    Eigen::Vector3d& aabb_max,
    const Eigen::Vector3d& point
) {
    aabb_min = aabb_min.cwiseMin(point);
    aabb_max = aabb_max.cwiseMax(point);
}

}  // namespace


void CropBox::updateTransform() {
    axis_aligned = rotation.isApprox(Eigen::Matrix3d::Identity(), kRotationEpsilon);

    const Eigen::Vector3d padded_min = min - padding;
    const Eigen::Vector3d padded_max = max + padding;

    if (axis_aligned) {
        world_to_box = Eigen::Isometry3d(Eigen::Translation3d(-origin));
        aabb_min = origin + padded_min;
        aabb_max = origin + padded_max;
        return;
    }

    Eigen::Isometry3d box_to_world = Eigen::Isometry3d::Identity();
    box_to_world.translation() = origin;
    box_to_world.linear() = rotation;
    world_to_box = box_to_world.inverse();

    aabb_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    aabb_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

    for (int i = 0; i < 8; ++i) {
        const Eigen::Vector3d corner(
            (i & 1) ? padded_max.x() : padded_min.x(),
            (i & 2) ? padded_max.y() : padded_min.y(),
            (i & 4) ? padded_max.z() : padded_min.z()
        );

        expand(aabb_min, aabb_max, box_to_world * corner);
    }
}


void CropBox::updateSweptAabb(bool dynamic) {
    const Eigen::Vector3d padded_min = min - padding;
    const Eigen::Vector3d padded_max = max + padding;

    if (!dynamic) {
        // Статический бокс не поворачивается, его swept совпадает с обычным AABB.
        swept_aabb_min = aabb_min;
        swept_aabb_max = aabb_max;
        return;
    }

    if (!has_angle_range) {
        double radius = 0.0;
        for (int i = 0; i < 8; ++i) {
            const Eigen::Vector3d corner(
                (i & 1) ? padded_max.x() : padded_min.x(),
                (i & 2) ? padded_max.y() : padded_min.y(),
                (i & 4) ? padded_max.z() : padded_min.z()
            );
            radius = std::max(radius, corner.norm());
        }

        swept_aabb_min = origin - Eigen::Vector3d::Constant(radius);
        swept_aabb_max = origin + Eigen::Vector3d::Constant(radius);
        return;
    }

    swept_aabb_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    swept_aabb_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

    const double step = (kSweptSamples > 1)
        ? (angle_max - angle_min) / (kSweptSamples - 1)
        : 0.0;

    for (int axis_index = 0; axis_index < 3; ++axis_index) {
        const Eigen::Vector3d axis = Eigen::Vector3d::Unit(axis_index);

        for (int s = 0; s < kSweptSamples; ++s) {
            const double angle = angle_min + step * s;
            const Eigen::Matrix3d sample_rotation =
                Eigen::AngleAxisd(angle, axis).toRotationMatrix();

            for (int i = 0; i < 8; ++i) {
                const Eigen::Vector3d corner(
                    (i & 1) ? padded_max.x() : padded_min.x(),
                    (i & 2) ? padded_max.y() : padded_min.y(),
                    (i & 4) ? padded_max.z() : padded_min.z()
                );

                expand(swept_aabb_min, swept_aabb_max, origin + sample_rotation * corner);
            }
        }
    }
}


void CropBox::applyFramePose(const Eigen::Isometry3d& frame_pose, bool dynamic) {
    origin = frame_pose.translation();
    rotation = frame_pose.linear();
    updateTransform();
    updateSweptAabb(dynamic);
}


bool CropBox::contains(const Eigen::Vector3d& point) const {
    if (!insideAabb(point)) {
        return false;
    }

    if (axis_aligned) {
        return true;
    }

    const Eigen::Vector3d local = world_to_box * point;

    return (local.array() >= (min - padding).array()).all()
        && (local.array() <= (max + padding).array()).all();
}

}  // namespace robot_body_filter
