#include "robot_body_filter/robot_body_filter_node.hpp"

#include <chrono>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


namespace robot_body_filter {

RobotBodyFilterNode::RobotBodyFilterNode()
    : rclcpp::Node("robot_body_filter_node") {

    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    tf_timeout_ = this->declare_parameter<double>("tf_timeout", 0.2);
    initial_tf_timeout_ = this->declare_parameter<double>("initial_tf_timeout", 10.0);
    // Вернуть отфильтрованное облако во фрейм входа (лидара): потребителям,
    // которые считают лучи от начала координат облака, нужен сам датчик.
    keep_input_frame_ = this->declare_parameter<bool>("keep_input_frame", false);

    filter_.setStaticBoxes(declareBoxes("static_boxes", false));
    filter_.setDynamicBoxes(declareBoxes("dynamic_boxes", true));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (!waitForInitialPoses()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "initial poses not available after %.1f s, static boxes stay unplaced",
            initial_tf_timeout_
        );
    }

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input/pointcloud",
        rclcpp::SensorDataQoS(),
        std::bind(&RobotBodyFilterNode::cloudCallback, this, std::placeholders::_1)
    );

    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/output/pointcloud",
        rclcpp::SensorDataQoS()
    );

    for (const auto& box : filter_.staticBoxes()) {
        crop_box_polygon_pubs_.push_back(
            this->create_publisher<PolygonStamped>("~/" + box.name + "/crop_box_polygon", 10)
        );
    }
    for (const auto& box : filter_.dynamicBoxes()) {
        crop_box_polygon_pubs_.push_back(
            this->create_publisher<PolygonStamped>("~/" + box.name + "/crop_box_polygon", 10)
        );
    }

    RCLCPP_INFO(
        this->get_logger(),
        "robot_body_filter: %zu static, %zu dynamic boxes in frame '%s'",
        filter_.staticBoxes().size(),
        filter_.dynamicBoxes().size(),
        base_frame_.c_str()
    );
}


std::vector<CropBox> RobotBodyFilterNode::declareBoxes(
    const std::string& prefix,
    bool dynamic
) {
    const auto names = this->declare_parameter<std::vector<std::string>>(
        prefix + ".names",
        std::vector<std::string>{}
    );

    std::vector<CropBox> boxes;
    boxes.reserve(names.size());

    for (const auto& name : names) {
        const std::string box_prefix = prefix + "." + name + ".";

        const auto frame = this->declare_parameter<std::string>(box_prefix + "frame", "");

        const auto min_x = this->declare_parameter<double>(box_prefix + "min_x", 0.0);
        const auto min_y = this->declare_parameter<double>(box_prefix + "min_y", 0.0);
        const auto min_z = this->declare_parameter<double>(box_prefix + "min_z", 0.0);
        const auto max_x = this->declare_parameter<double>(box_prefix + "max_x", 0.0);
        const auto max_y = this->declare_parameter<double>(box_prefix + "max_y", 0.0);
        const auto max_z = this->declare_parameter<double>(box_prefix + "max_z", 0.0);

        const auto padding_x = this->declare_parameter<double>(box_prefix + "padding_x", 0.0);
        const auto padding_y = this->declare_parameter<double>(box_prefix + "padding_y", 0.0);
        const auto padding_z = this->declare_parameter<double>(box_prefix + "padding_z", 0.0);

        if (frame.empty()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "box '%s': frame is required, skipped",
                name.c_str()
            );
            continue;
        }

        CropBox box;
        box.name = name;
        box.frame = frame;
        box.min = Eigen::Vector3d(min_x, min_y, min_z);
        box.max = Eigen::Vector3d(max_x, max_y, max_z);
        box.padding = Eigen::Vector3d(padding_x, padding_y, padding_z);

        if ((box.min.array() > box.max.array()).any()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "box '%s': min must not exceed max, skipped",
                name.c_str()
            );
            continue;
        }

        if (dynamic) {
            const auto use_angle_range =
                this->declare_parameter<bool>(box_prefix + "use_angle_range", false);
            const auto angle_min =
                this->declare_parameter<double>(box_prefix + "angle_min", 0.0);
            const auto angle_max =
                this->declare_parameter<double>(box_prefix + "angle_max", 0.0);

            if (use_angle_range) {
                if (angle_min > angle_max) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "box '%s': angle_min must not exceed angle_max, skipped",
                        name.c_str()
                    );
                    continue;
                }

                box.has_angle_range = true;
                box.angle_min = angle_min;
                box.angle_max = angle_max;
            }
        }

        boxes.push_back(box);
    }

    return boxes;
}


void RobotBodyFilterNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg
) {
    if (!msg->is_dense) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "input cloud is not dense, invalid points are passed through"
        );
    }

    // Статические боксы уже стоят с момента старта — на каждом облаке
    // разрешаются только подвижные звенья.
    std::vector<Eigen::Isometry3d> joint_poses;
    if (!lookupFramePoses(filter_.jointFrames(), msg->header.stamp, joint_poses)) {
        return;
    }
    filter_.updateDynamicBoxes(joint_poses);

    sensor_msgs::msg::PointCloud2 cloud_in_base;
    if (!transformCloudToBaseFrame(*msg, cloud_in_base)) {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud_filtered;
    const FilterStats stats = filter_.filter(cloud_in_base, cloud_filtered);

    if (keep_input_frame_ && msg->header.frame_id != base_frame_) {
        geometry_msgs::msg::TransformStamped back;
        try {
            back = tf_buffer_->lookupTransform(
                msg->header.frame_id,
                base_frame_,
                msg->header.stamp,
                tf2::durationFromSec(tf_timeout_)
            );
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "cannot transform '%s' -> '%s': %s",
                base_frame_.c_str(),
                msg->header.frame_id.c_str(),
                ex.what()
            );
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud_in_input;
        tf2::doTransform(cloud_filtered, cloud_in_input, back);
        filtered_cloud_pub_->publish(cloud_in_input);
    } else {
        filtered_cloud_pub_->publish(cloud_filtered);
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "in %zu, removed %zu, out %zu",
        stats.pointsIn(),
        stats.pointsRemoved(),
        stats.pointsOut()
    );

    publishCropBoxPolygons();
}


bool RobotBodyFilterNode::lookupFramePoses(
    const std::vector<std::string>& frames,
    const rclcpp::Time& stamp,
    std::vector<Eigen::Isometry3d>& poses
) {
    poses.clear();
    poses.reserve(frames.size());

    for (const auto& frame : frames) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                base_frame_,
                frame,
                stamp,
                tf2::durationFromSec(tf_timeout_)
            );
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "cannot transform '%s' -> '%s': %s",
                frame.c_str(),
                base_frame_.c_str(),
                ex.what()
            );
            return false;
        }

        poses.push_back(tf2::transformToEigen(transform));
    }

    return true;
}


bool RobotBodyFilterNode::waitForInitialPoses() {
    const auto static_frames = filter_.staticFrames();
    const auto joint_frames = filter_.jointFrames();

    if (static_frames.empty() && joint_frames.empty()) {
        return true;
    }

    const rclcpp::Time deadline = this->now() + rclcpp::Duration::from_seconds(
        initial_tf_timeout_
    );

    // Нулевой stamp — последняя доступная поза: на старте точное время неважно,
    // нужны сами звенья.
    const rclcpp::Time latest(0, 0, this->get_clock()->get_clock_type());

    std::vector<Eigen::Isometry3d> static_poses;
    std::vector<Eigen::Isometry3d> joint_poses;

    while (rclcpp::ok() && this->now() < deadline) {
        if (lookupFramePoses(static_frames, latest, static_poses)
            && lookupFramePoses(joint_frames, latest, joint_poses)) {

            // Статические боксы ставятся один раз: их звенья неподвижны
            // относительно base_frame, и на каждом облаке TF для них не нужен.
            filter_.updateStaticBoxes(static_poses);
            filter_.updateDynamicBoxes(joint_poses);

            RCLCPP_INFO(
                this->get_logger(),
                "initial poses resolved: %zu static, %zu dynamic boxes",
                static_frames.size(),
                joint_frames.size()
            );
            return true;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    return false;
}


bool RobotBodyFilterNode::transformCloudToBaseFrame(
    const sensor_msgs::msg::PointCloud2& cloud_in,
    sensor_msgs::msg::PointCloud2& cloud_out
) {
    if (cloud_in.header.frame_id == base_frame_) {
        cloud_out = cloud_in;
        return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            base_frame_,
            cloud_in.header.frame_id,
            cloud_in.header.stamp,
            tf2::durationFromSec(tf_timeout_)
        );
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "cannot transform '%s' -> '%s': %s",
            cloud_in.header.frame_id.c_str(),
            base_frame_.c_str(),
            ex.what()
        );
        return false;
    }

    tf2::doTransform(cloud_in, cloud_out, transform);
    return true;
}


void RobotBodyFilterNode::publishCropBoxPolygons() {
    size_t index = 0;

    const auto publish = [this, &index](const std::vector<CropBox>& boxes) {
        for (const auto& box : boxes) {
            if (index >= crop_box_polygon_pubs_.size()) {
                return;
            }

            const Eigen::Vector3d lo = box.min - box.padding;
            const Eigen::Vector3d hi = box.max + box.padding;

            // Бокс задан в СК своего звена и может быть повёрнут, поэтому углы
            // переводятся в base_frame. У autoware бокс осепараллельный и там
            // координаты берутся из параметров напрямую.
            Eigen::Isometry3d box_to_base = Eigen::Isometry3d::Identity();
            box_to_base.translation() = box.origin;
            box_to_base.linear() = box.rotation;

            const auto point = [&box_to_base](double x, double y, double z) {
                const Eigen::Vector3d p = box_to_base * Eigen::Vector3d(x, y, z);
                geometry_msgs::msg::Point32 msg;
                msg.x = static_cast<float>(p.x());
                msg.y = static_cast<float>(p.y());
                msg.z = static_cast<float>(p.z());
                return msg;
            };

            const double x1 = hi.x();
            const double x2 = lo.x();
            const double x3 = lo.x();
            const double x4 = hi.x();

            const double y1 = hi.y();
            const double y2 = hi.y();
            const double y3 = lo.y();
            const double y4 = lo.y();

            const double z1 = lo.z();
            const double z2 = hi.z();

            PolygonStamped polygon_msg;
            polygon_msg.header.frame_id = base_frame_;
            polygon_msg.header.stamp = this->get_clock()->now();

            auto& points = polygon_msg.polygon.points;
            points.push_back(point(x1, y1, z1));
            points.push_back(point(x2, y2, z1));
            points.push_back(point(x3, y3, z1));
            points.push_back(point(x4, y4, z1));
            points.push_back(point(x1, y1, z1));

            points.push_back(point(x1, y1, z2));

            points.push_back(point(x2, y2, z2));
            points.push_back(point(x2, y2, z1));
            points.push_back(point(x2, y2, z2));

            points.push_back(point(x3, y3, z2));
            points.push_back(point(x3, y3, z1));
            points.push_back(point(x3, y3, z2));

            points.push_back(point(x4, y4, z2));
            points.push_back(point(x4, y4, z1));
            points.push_back(point(x4, y4, z2));

            points.push_back(point(x1, y1, z2));

            crop_box_polygon_pubs_[index]->publish(polygon_msg);
            ++index;
        }
    };

    publish(filter_.staticBoxes());
    publish(filter_.dynamicBoxes());
}


}  // namespace robot_body_filter


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_body_filter::RobotBodyFilterNode>());
    rclcpp::shutdown();
    return 0;
}
