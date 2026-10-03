// SPDX-License-Identifier: Apache-2.0
#include <array>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "hsl_perception_cpp/detector.hpp"
using namespace hsl_perception_cpp;
class OpponentDetector : public rclcpp::Node {
public:
  OpponentDetector():Node("opponent_detector"){
    auto&m=detector_.model;auto&t=detector_.tracking;auto&c=detector_.checks;
#define MODEL(key) m.key=declare_parameter("robot." #key,m.key)
    MODEL(radius);MODEL(rim_max_z);MODEL(max_height);MODEL(min_top);MODEL(max_extent);MODEL(min_extent);MODEL(gap_min_z);MODEL(gap_max_z);MODEL(max_gap_share);MODEL(contain_margin);MODEL(min_points);MODEL(min_rim_points);MODEL(outlier);MODEL(fit_rms_max);MODEL(line_ratio);MODEL(measurement_std);MODEL(fallback_std);
#undef MODEL
#define TRACK(key) t.key=declare_parameter("tracker." #key,t.key)
    TRACK(accel_std);TRACK(position_std);TRACK(initial_speed_std);TRACK(heading_speed);TRACK(omega_time);TRACK(gate);TRACK(confirm_hits);TRACK(max_tracks);TRACK(move_min_hits);TRACK(tentative_coast);TRACK(max_coast);TRACK(move_threshold);TRACK(reacquire_radius);
#undef TRACK
#define CHECK(key) c.key=declare_parameter(#key,c.key)
    CHECK(strong_arc_min_span_deg);CHECK(strong_min_inlier_fraction);CHECK(strong_min_extent);CHECK(strong_rectangle_ratio);CHECK(allow_merged_strong);
#undef CHECK
    m.max_height=declare_parameter("opponent_max_height",.46);
    if(!std::isfinite(m.max_height)||m.max_height<.08||m.max_height>.60||m.radius<=0||t.confirm_hits<1||t.omega_time<=0)throw std::invalid_argument("invalid detector geometry/tracking parameters");
    auto prefix=std::string(get_namespace());sensor_frame_=declare_parameter<std::string>("sensor_frame",prefix=="/"?"livox_frame":prefix.substr(1)+"/livox_frame");
    tf_=std::make_shared<tf2_ros::Buffer>(get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);
    grid_=create_subscription<nav_msgs::msg::OccupancyGrid>("navigation/known_grid",rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      if(m->header.frame_id!="map"||!std::isfinite(m->info.resolution)||m->info.resolution<=0||!std::isfinite(m->info.origin.position.x)||!std::isfinite(m->info.origin.position.y)||m->info.width==0||m->data.size()!=static_cast<size_t>(m->info.width)*m->info.height)return;
      detector_.background.set(m->info.resolution,m->info.origin.position.x,m->info.origin.position.y,m->info.width,m->info.height,std::vector<int>(m->data.begin(),m->data.end()));has_grid_=true;});
    own_sub_=create_subscription<nav_msgs::msg::Odometry>("navigation/self",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){if(m->header.frame_id=="map")own_=m;});
    scan_=create_subscription<sensor_msgs::msg::PointCloud2>("navigation/scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
      if(!pending_.empty()&&rclcpp::Time(m->header.stamp)<rclcpp::Time(pending_.back()->header.stamp)){pending_.clear();visible_stamp_=-INFINITY;}
      if(pending_.size()==8)pending_.pop_front();pending_.push_back(m);});
    opponent_=create_publisher<nav_msgs::msg::Odometry>("navigation/opponent",10);visible_=create_publisher<std_msgs::msg::Bool>("navigation/opponent_visible",10);timing_=create_publisher<std_msgs::msg::Float32>("navigation/detector_cycle_ms",10);diag_=create_publisher<std_msgs::msg::String>("navigation/detector_diagnostics",10);
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{flush();});visible_timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{double age=now().seconds()-visible_stamp_;std_msgs::msg::Bool m;m.data=age>=0&&age<=.3;visible_->publish(m);});
  }
private:
  Points read(const sensor_msgs::msg::PointCloud2&m){
    std::array<int,3> offsets{-1,-1,-1};for(auto&f:m.fields)for(size_t i=0;i<3;++i)if(f.name==std::array<const char*,3>{"x","y","z"}[i]&&f.datatype==7&&f.count==1&&static_cast<uint64_t>(f.offset)+4<=m.point_step)offsets[i]=f.offset;
    if(std::any_of(offsets.begin(),offsets.end(),[](int o){return o<0;})||m.point_step==0||static_cast<uint64_t>(m.width)*m.point_step>m.row_step||static_cast<uint64_t>(m.height)*m.row_step>m.data.size())throw std::runtime_error("invalid scan layout");
    Points result;result.reserve(static_cast<size_t>(m.width)*m.height);
    for(uint32_t row=0;row<m.height;++row)for(uint32_t col=0;col<m.width;++col){V3 p;size_t base=static_cast<size_t>(row)*m.row_step+static_cast<size_t>(col)*m.point_step;
      for(size_t i=0;i<3;++i){uint32_t bits=0;for(int j=0;j<4;++j)bits|=static_cast<uint32_t>(m.data[base+offsets[i]+j])<<(8*(m.is_bigendian?3-j:j));float v;std::memcpy(&v,&bits,4);p[i]=v;}if(p.allFinite())result.push_back(p);
    }return result;
  }
  void flush(){if(pending_.empty())return;auto msg=pending_.front();double stamp=rclcpp::Time(msg->header.stamp).seconds(),age=now().seconds()-stamp;
    if(age<0||age>.5){pending_.pop_front();return;}
    if(!has_grid_||!own_||msg->header.frame_id!="map")return;
    // Own pose is a readiness check, not the geometric transform of this scan.
    // Geometry comes exclusively from the sensor TF at the measurement stamp.
    double own_age=now().seconds()-rclcpp::Time(own_->header.stamp).seconds();
    if(own_age<0||own_age>1.2)return;
    geometry_msgs::msg::TransformStamped transform;try{transform=tf_->lookupTransform("map",sensor_frame_,rclcpp::Time(msg->header.stamp));}catch(const tf2::TransformException&){return;}
    pending_.pop_front();auto started=std::chrono::steady_clock::now();Points points;try{points=read(*msg);}catch(const std::exception&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"%s",e.what());return;}
    auto&s=transform.transform.translation;auto track=detector_.step(points,{s.x,s.y,s.z},stamp);bool detected=track&&std::abs(track->last_update-stamp)<1e-6;
    if(detected){visible_stamp_=stamp;nav_msgs::msg::Odometry out;out.header=msg->header;out.child_frame_id="tracked_opponent/base_footprint";out.pose.pose.position.x=track->mean[0];out.pose.pose.position.y=track->mean[1];double yaw=track->heading;out.pose.pose.orientation.z=std::sin(yaw/2);out.pose.pose.orientation.w=std::cos(yaw/2);
      out.twist.twist.linear.x=std::cos(yaw)*track->mean[2]+std::sin(yaw)*track->mean[3];out.twist.twist.linear.y=-std::sin(yaw)*track->mean[2]+std::cos(yaw)*track->mean[3];out.twist.twist.angular.z=track->omega;
      out.pose.covariance[0]=track->covariance(0,0);out.pose.covariance[1]=track->covariance(0,1);out.pose.covariance[6]=track->covariance(1,0);out.pose.covariance[7]=track->covariance(1,1);V2 across{-std::sin(yaw),std::cos(yaw)};double variance=(across.transpose()*track->covariance.block<2,2>(2,2)*across)(0,0);double speed2=track->mean.tail<2>().squaredNorm();out.pose.covariance[35]=track->heading_known?std::min(variance/std::max(speed2,1e-9),pi*pi):pi*pi;opponent_->publish(out);}
    double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();auto&a=detector_.diagnostics;std::ostringstream text;text<<std::setprecision(16)<<"{\"stamp_s\":"<<stamp<<",\"cycle_ms\":"<<ms<<",\"foreground_points\":"<<a.foreground_points<<",\"clusters\":"<<a.clusters<<",\"candidates\":"<<a.candidates<<",\"strong_candidates\":"<<a.strong_candidates<<",\"tracks\":"<<a.tracks<<",\"detected\":"<<(detected?"true":"false")<<",\"coasting\":"<<(track&&!detected?"true":"false")<<",\"track_xy\":";
    if(track)text<<"["<<track->mean[0]<<","<<track->mean[1]<<"]";else text<<"null";auto counts=[&](const std::map<std::string,size_t>&values){text<<"{";bool first=true;for(auto&[key,count]:values){if(!first)text<<",";first=false;text<<"\""<<key<<"\":"<<count;}text<<"}";};text<<",\"rejections\":";counts(a.rejections);text<<",\"weak_reasons\":";counts(a.weak_reasons);text<<"}";std_msgs::msg::String d;d.data=text.str();diag_->publish(d);std_msgs::msg::Float32 elapsed;elapsed.data=ms;timing_->publish(elapsed);
  }
  Detector detector_;std::string sensor_frame_;bool has_grid_=false;double visible_stamp_=-INFINITY;
  nav_msgs::msg::Odometry::ConstSharedPtr own_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr>pending_;
  std::shared_ptr<tf2_ros::Buffer>tf_;std::shared_ptr<tf2_ros::TransformListener>listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_sub_;rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr opponent_;rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_;rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr timing_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_;rclcpp::TimerBase::SharedPtr timer_,visible_timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<OpponentDetector>());}catch(const std::exception&e){fprintf(stderr,"%s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
