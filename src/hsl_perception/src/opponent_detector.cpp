// Copyright 2026 Just Robotics. SPDX-License-Identifier: Apache-2.0
#include "hsl_perception/detector.hpp"
#include <array>
#include <chrono>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

class OpponentDetector: public rclcpp::Node {
public:
  OpponentDetector():Node("opponent_detector") {
    max_opponent_height_=declare_parameter<double>("opponent_max_height",0.46);
    if (!std::isfinite(max_opponent_height_) || max_opponent_height_<0.08 || max_opponent_height_>0.60) {
      throw std::invalid_argument("opponent_max_height must be in [0.08,0.60] map-frame metres");
    }
    opponent_pub_=create_publisher<nav_msgs::msg::Odometry>("navigation/opponent",10);
    visible_pub_=create_publisher<std_msgs::msg::Bool>("navigation/opponent_visible",10);
    timing_pub_=create_publisher<std_msgs::msg::Float32>("navigation/detector_cycle_ms",10);
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("navigation/known_grid",
      rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        if (m->header.frame_id!="map") {return;}
        grid_={m->info.resolution,m->info.origin.position.x,m->info.origin.position.y,
          int(m->info.width),int(m->info.height),m->data};
      });
    self_sub_=create_subscription<nav_msgs::msg::Odometry>("navigation/self",10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {if (m->header.frame_id=="map") {own_=m;}});
    scan_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("navigation/scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {scan(*m);});
    timer_=rclcpp::create_timer(this,get_clock(),std::chrono::milliseconds(50),[this]() {
      const double age=now().seconds()-last_seen_;
      std_msgs::msg::Bool msg; msg.data=visible_ && age>=0 && age<=0.3;
      visible_pub_->publish(msg);
    });
  }
private:
  void scan(const sensor_msgs::msg::PointCloud2 & msg) {
    if (!own_ || !grid_.valid() || msg.header.frame_id!="map") {return;}
    const auto started=std::chrono::steady_clock::now();
    std::array<uint32_t,3> offsets{}; std::array<bool,3> found{};
    for (const auto & field:msg.fields) {
      for (size_t i=0;i<3;++i) {
        if (field.name==std::array<std::string,3>{"x","y","z"}[i] &&
          field.datatype==sensor_msgs::msg::PointField::FLOAT32 && field.offset+4<=msg.point_step) {
          offsets[i]=field.offset; found[i]=true;
        }
      }
    }
    if (!std::all_of(found.begin(),found.end(),[](bool v) {return v;})) {return;}
    std::vector<hsl_perception::Point> points;
    points.reserve(size_t(msg.width)*msg.height);
    for (uint32_t row=0;row<msg.height;++row) {
      for (uint32_t col=0;col<msg.width;++col) {
        const size_t base=size_t(row)*msg.row_step+size_t(col)*msg.point_step;
        if (base+msg.point_step>msg.data.size()) {continue;}
        std::array<float,3> xyz{};
        for (size_t i=0;i<3;++i) {
          std::array<uint8_t,4> bytes{};
          std::copy_n(msg.data.data()+base+offsets[i],4,bytes.begin());
          const uint16_t endian=1;
          const bool host_big=*reinterpret_cast<const uint8_t *>(&endian)==0;
          if (msg.is_bigendian!=host_big) {std::reverse(bytes.begin(),bytes.end());}
          std::memcpy(&xyz[i],bytes.data(),4);
        }
        points.push_back({xyz[0],xyz[1],xyz[2]});
      }
    }
    const double t=now().seconds();
    const auto previous=(previous_ && t-last_seen_>=0 && t-last_seen_<=1.0) ? previous_ : std::nullopt;
    const auto & position=own_->pose.pose.position;
    const auto result=hsl_perception::detect(points,grid_,{position.x,position.y},previous,max_opponent_height_);
    visible_=bool(result);
    if (result) {
      const auto p=result->centre;
      const double dt=t-last_seen_;
      if (previous_ && dt>=0.05 && dt<=1.0) {
        vx_=0.5*vx_+0.5*(p.x-previous_->x)/dt;
        vy_=0.5*vy_+0.5*(p.y-previous_->y)/dt;
      } else if (!previous_ || dt>1.0 || dt<0) {vx_=vy_=0;}
      previous_=p; last_seen_=t;
      const double yaw=std::hypot(vx_,vy_)>0.03 ? std::atan2(vy_,vx_) : 0;
      nav_msgs::msg::Odometry enemy;
      enemy.header=msg.header; enemy.child_frame_id="opponent/base_footprint";
      enemy.pose.pose.position.x=p.x; enemy.pose.pose.position.y=p.y;
      enemy.pose.pose.orientation.z=std::sin(yaw/2); enemy.pose.pose.orientation.w=std::cos(yaw/2);
      enemy.twist.twist.linear.x=std::cos(yaw)*vx_+std::sin(yaw)*vy_;
      enemy.twist.twist.linear.y=-std::sin(yaw)*vx_+std::cos(yaw)*vy_;
      opponent_pub_->publish(enemy);
    }
    std_msgs::msg::Float32 timing;
    timing.data=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-started).count();
    timing_pub_->publish(timing);
  }
  hsl_perception::Grid grid_;
  nav_msgs::msg::Odometry::SharedPtr own_;
  std::optional<hsl_perception::Position> previous_;
  double max_opponent_height_=0.46;
  double last_seen_=-1e20,vx_=0,vy_=0;
  bool visible_=false;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr self_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr opponent_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr timing_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<OpponentDetector>()); rclcpp::shutdown();
}
