// SPDX-License-Identifier: Apache-2.0
// Native adaptation of feature/detector circle geometry and CV Kalman tracking.
#pragma once
#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>
namespace hsl_perception_cpp
{
using V2=Eigen::Vector2d;
using V3=Eigen::Vector3d;
using Points=std::vector<V3>;
constexpr double pi=3.14159265358979323846;
inline double wrap(double a){a=std::fmod(a+pi,2*pi);if(a<0)a+=2*pi;return a-pi;}
inline double quantile(std::vector<double> v,double q){
  if(v.empty())return 0.;std::sort(v.begin(),v.end());double i=q*(v.size()-1);
  auto lo=static_cast<size_t>(i);auto hi=std::min(lo+1,v.size()-1);return v[lo]+(i-lo)*(v[hi]-v[lo]);}
struct Model {
  double radius=.178,rim_max_z=.115,max_height=.46,min_top=0,max_extent=.42,min_extent=0;
  double gap_min_z=.25,gap_max_z=.34,max_gap_share=1,contain_margin=.05;
  int min_points=3,min_rim_points=4;
  double outlier=.05,fit_rms_max=.035,line_ratio=.7,measurement_std=.03,fallback_std=.1;
};
struct Tracking {
  double accel_std=.5,position_std=.01,initial_speed_std=.5,heading_speed=.05,omega_time=.5,gate=9.21;
  int confirm_hits=3,max_tracks=8,move_min_hits=10;
  double tentative_coast=.35,max_coast=1.5,move_threshold=.3,reacquire_radius=.35;
};
struct Checks {double strong_arc_min_span_deg=0,strong_min_inlier_fraction=0,strong_min_extent=0,strong_rectangle_ratio=0;bool allow_merged_strong=true;};
struct Detection {V2 center;double sigma;bool strong;};
struct Background {
  double resolution=.05,ox=0,oy=0;int width=0,height=0;
  std::vector<int> grid;std::vector<bool> mask;
  void set(double r,double x,double y,int w,int h,const std::vector<int>&data){
    resolution=r;ox=x;oy=y;width=w;height=h;grid=data;mask.assign(grid.size(),false);
    int radius=static_cast<int>(std::ceil(.08/r+.71));
    for(int yy=0;yy<h;++yy)for(int xx=0;xx<w;++xx)if(grid[yy*w+xx]>=50){
      for(int dy=-radius;dy<=radius;++dy)for(int dx=-radius;dx<=radius;++dx){
        int nx=xx+dx,ny=yy+dy;
        if(nx>=0&&ny>=0&&nx<w&&ny<h&&std::hypot(dx,dy)*r<=.08+.71*r)mask[ny*w+nx]=true;
      }}
  }
  int index(const V2&p)const{int x=std::floor((p.x()-ox)/resolution),y=std::floor((p.y()-oy)/resolution);return x<0||y<0||x>=width||y>=height?-1:y*width+x;}
  bool foreground(const V2&p)const{int i=index(p);return i>=0&&grid[i]>=0&&!mask[i];}
  bool free(const V2&p)const{int i=index(p);return i>=0&&grid[i]>=0&&grid[i]<50;}
};
inline std::vector<Points> cluster(const Points&p,double cell=.1){
  using Cell=std::pair<int,int>;std::map<Cell,std::vector<size_t>> bins;
  for(size_t i=0;i<p.size();++i)bins[{std::floor(p[i].x()/cell),std::floor(p[i].y()/cell)}].push_back(i);
  std::set<Cell>seen;std::vector<Points>result;
  for(auto &item:bins){if(seen.count(item.first))continue;
    std::vector<Cell>stack{item.first};seen.insert(item.first);std::vector<size_t>ids;
    while(!stack.empty()){Cell c=stack.back();stack.pop_back();auto&indices=bins.at(c);ids.insert(ids.end(),indices.begin(),indices.end());
      for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy){Cell next{c.first+dx,c.second+dy};
        if(bins.count(next)&&!seen.count(next)){seen.insert(next);stack.push_back(next);}}}
    std::sort(ids.begin(),ids.end());Points group;for(auto i:ids)group.push_back(p[i]);result.push_back(group);
  }return result;
}
struct Axes {Eigen::Matrix2d vectors;Eigen::Vector2d values;V2 mean;};
inline Axes axes(const Points&p){V2 mean=V2::Zero();for(auto&v:p)mean+=v.head<2>();mean/=p.size();
  Eigen::Matrix2d c=Eigen::Matrix2d::Zero();for(auto&v:p){V2 d=v.head<2>()-mean;c+=d*d.transpose();}c/=p.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>s(c);return {s.eigenvectors(),s.eigenvalues(),mean};}
inline double extent(const Points&p){if(p.empty())return 0;auto a=axes(p);std::vector<double>v;for(auto&pt:p)v.push_back((pt.head<2>()-a.mean).dot(a.vectors.col(1)));return quantile(v,.98)-quantile(v,.02);}
inline double line_rms(const Points&p){return std::sqrt(std::max(0.,axes(p).values[0]));}
inline V2 edge_center(const Points&p,const Points&rim,const V2&observer,double radius){
  V2 mean=V2::Zero();for(auto&v:p)mean+=v.head<2>()-observer;mean/=p.size();double base=std::atan2(mean.y(),mean.x()),lo=pi,hi=-pi;
  for(auto&v:p){V2 d=v.head<2>()-observer;double a=wrap(std::atan2(d.y(),d.x())-base);lo=std::min(lo,a);hi=std::max(hi,a);}
  double heading=base+(lo+hi)/2.;V2 dir{std::cos(heading),std::sin(heading)},across{-dir.y(),dir.x()};std::vector<double>dist;
  for(auto&v:rim.empty()?p:rim){V2 d=v.head<2>()-observer;double r=d.dot(dir);if(!rim.empty())r+=std::sqrt(std::max(0.,radius*radius-std::pow(d.dot(across),2)));dist.push_back(r);}
  return observer+quantile(dist,.5)*dir;
}
inline V2 fit_circle(const Points&p,double radius,V2 center){
  for(int iteration=0;iteration<10;++iteration){Eigen::Matrix2d lhs=Eigen::Matrix2d::Identity()*1e-9;V2 rhs=V2::Zero();
    for(auto&v:p){V2 d=v.head<2>()-center;double distance=std::max(d.norm(),1e-9);V2 unit=d/distance;lhs+=unit*unit.transpose();rhs+=unit*(distance-radius);}
    V2 step=lhs.ldlt().solve(rhs);double length=step.norm();if(length>radius)step*=radius/length;center+=step;if(length<1e-4)break;
  }return center;
}
inline double rectangle_mse(const Points&p){if(p.size()<4)return INFINITY;double best=INFINITY;
  for(int i=0;i<=45;++i){double a=pi/2*i/45.;V2 x{std::cos(a),std::sin(a)},y{-x.y(),x.x()};std::vector<double>xs,ys;
    for(auto&v:p){xs.push_back(v.head<2>().dot(x));ys.push_back(v.head<2>().dot(y));}
    double lx=quantile(xs,.05),hx=quantile(xs,.95),ly=quantile(ys,.05),hy=quantile(ys,.95),sum=0;
    for(size_t j=0;j<p.size();++j){double e=std::min({std::abs(xs[j]-lx),std::abs(xs[j]-hx),std::abs(ys[j]-ly),std::abs(ys[j]-hy)});sum+=e*e;}best=std::min(best,sum/p.size());
  }return best;
}
inline std::optional<Detection> inspect(const Points&p,const V2&observer,const Model&m,bool extract=true){
  if(p.size()<static_cast<size_t>(m.min_points))return {};
  double span=extent(p);
  if(span>m.max_extent){if(!extract)return {};Points rim;for(auto&v:p)if(v.z()<=m.rim_max_z)rim.push_back(v);
    if(rim.size()<3)return {};uint32_t state=static_cast<uint32_t>(rim.size());std::array<size_t,400>pairs;
    for(auto&index:pairs){state^=state<<13;state^=state>>17;state^=state<<5;index=state%rim.size();}
    V2 best=V2::Zero();size_t count=0;
    // Same seeded xorshift32 pairs and ordering as the offline Python oracle.
    for(int i=0;i<200;++i){V2 a=rim[pairs[i]].head<2>(),b=rim[pairs[200+i]].head<2>();V2 chord=b-a;double half=chord.norm()/2;if(half<=.02||half>=m.radius)continue;
      V2 middle=(a+b)/2,normal{-chord.y(),chord.x()};normal/=2*half;V2 delta=std::sqrt(m.radius*m.radius-half*half)*normal;
      V2 c1=middle-delta,c2=middle+delta,c=(c2-observer).norm()>=(c1-observer).norm()?c2:c1;size_t hits=0;
      for(auto&v:rim)hits+=std::abs((v.head<2>()-c).norm()-m.radius)<=m.outlier/2;
      if(hits>count){count=hits;best=c;}}
    if(count<static_cast<size_t>(m.min_rim_points))return {};Points inside;for(auto&v:p)if((v.head<2>()-best).norm()<=m.radius+m.contain_margin)inside.push_back(v);return inspect(inside,observer,m,false);
  }
  double top=-INFINITY,gap=0;Points rim;
  for(auto&v:p){top=std::max(top,v.z());gap+=v.z()>=m.gap_min_z&&v.z()<m.gap_max_z;if(v.z()<=m.rim_max_z)rim.push_back(v);}
  if(top>m.max_height||top<m.min_top||span<m.min_extent||gap/p.size()>m.max_gap_share)return {};
  V2 guess=edge_center(p,rim,observer,m.radius);if(rim.size()<static_cast<size_t>(m.min_rim_points))return Detection{guess,m.fallback_std,false};
  V2 center=fit_circle(rim,m.radius,guess);Points arc;
  for(auto&v:rim)if(std::abs((v.head<2>()-center).norm()-m.radius)<=m.outlier)arc.push_back(v);
  if(arc.size()<static_cast<size_t>(m.min_rim_points))return {};
  if(arc.size()!=rim.size())center=fit_circle(arc,m.radius,center);
  double sum=0;for(auto&v:arc)sum+=std::pow((v.head<2>()-center).norm()-m.radius,2);double rms=std::sqrt(sum/arc.size());
  if(rms>m.fit_rms_max)return {};
  if(rms>m.line_ratio*line_rms(arc)){if(m.min_top>0||m.max_gap_share<1)return Detection{guess,m.fallback_std,false};return {};}
  return Detection{center,m.measurement_std,true};
}
struct Track {
  Eigen::Vector4d mean;Eigen::Matrix4d covariance;
  double heading=0,omega=0,time=0,last_update=0,last_strong_update=0,heading_time=0,travel=0;
  bool heading_known=false;V2 birth,last_position,origin;int hits=1,strong_hits=0;
  explicit Track(double t,const Detection&d,const Tracking&c):time(t),last_update(t),last_strong_update(d.strong?t:-INFINITY),heading_time(t),birth(d.center),last_position(d.center),origin(d.center),strong_hits(d.strong){
    mean<<d.center.x(),d.center.y(),0,0;covariance=Eigen::Matrix4d::Zero();covariance.diagonal()<<d.sigma*d.sigma,d.sigma*d.sigma,c.initial_speed_std*c.initial_speed_std,c.initial_speed_std*c.initial_speed_std;
  }
  bool confirmed(const Tracking&c)const{return strong_hits>=c.confirm_hits;}
  bool moved(const Tracking&c)const{return strong_hits>=c.move_min_hits&&travel>=c.move_threshold;}
  bool moving(const Tracking&c)const{double speed=mean.tail<2>().norm();if(speed<c.heading_speed)return false;V2 along=mean.tail<2>()/speed;double v=(along.transpose()*covariance.block<2,2>(2,2)*along)(0,0);return speed>2*std::sqrt(std::max(0.,v));}
  void predict(double t,const Tracking&c){double dt=t-time;if(dt<=0)return;time=t;Eigen::Matrix4d f=Eigen::Matrix4d::Identity(),q=Eigen::Matrix4d::Zero();f(0,2)=f(1,3)=dt;
    double a=c.accel_std*c.accel_std,p=c.position_std*c.position_std*dt;
    for(int i=0;i<2;++i){q(i,i)=a*std::pow(dt,4)/4+p;q(i,i+2)=q(i+2,i)=a*std::pow(dt,3)/2;q(i+2,i+2)=a*dt*dt;}
    mean=(f*mean).eval();covariance=(f*covariance*f.transpose()+q).eval();if(!moving(c))omega*=std::exp(-dt/c.omega_time);
  }
  double distance(const Detection&d)const{V2 innovation=d.center-mean.head<2>();Eigen::Matrix2d s=covariance.block<2,2>(0,0)+Eigen::Matrix2d::Identity()*d.sigma*d.sigma;return innovation.dot(s.ldlt().solve(innovation));}
  void update(double t,const Detection&d,const Tracking&c){Eigen::Matrix2d r=Eigen::Matrix2d::Identity()*d.sigma*d.sigma,s=covariance.block<2,2>(0,0)+r;
    Eigen::Matrix<double,4,2> k=covariance.block<4,2>(0,0)*s.inverse();mean+=(k*(d.center-mean.head<2>())).eval();Eigen::Matrix4d correction=Eigen::Matrix4d::Identity();correction.block<4,2>(0,0)-=k;
    covariance=(correction*covariance*correction.transpose()+k*r*k.transpose()).eval();
    if(moving(c)){double next=std::atan2(mean[3],mean[2]),dt=t-heading_time;if(heading_known&&dt>0){double rate=wrap(next-heading)/dt;omega+=dt/(c.omega_time+dt)*(rate-omega);}heading=next;heading_known=true;heading_time=t;}
    ++hits;strong_hits+=d.strong;last_update=t;if(d.strong)last_strong_update=t;last_position=mean.head<2>();if(d.strong)travel=std::max(travel,(d.center-origin).norm());
  }
};
struct Diagnostics {size_t foreground_points=0,clusters=0,candidates=0,strong_candidates=0,tracks=0;std::map<std::string,size_t> rejections,weak_reasons;};
class Detector {
public:
  Model model;Tracking tracking;Checks checks;Background background;Diagnostics diagnostics;
  std::vector<std::shared_ptr<Track>>tracks;std::shared_ptr<Track>selected;std::optional<V2>robot_position;double time=-INFINITY;
  std::shared_ptr<Track> step(const Points&points,const V3&sensor,double stamp){
    diagnostics={};Points foreground;
    for(auto&p:points){if(!p.allFinite())continue;double distance=std::max((p-sensor).norm(),1e-6),slope=std::clamp((sensor.z()-p.z())/distance,0.,1.);
      if(p.z()>=.012+.06*slope&&p.z()<=.70&&distance>=.30&&background.foreground(p.head<2>()))foreground.push_back(p);}
    auto groups=cluster(foreground);diagnostics.foreground_points=foreground.size();diagnostics.clusters=groups.size();std::vector<Detection>detections;
    for(auto&group:groups){auto det=inspect(group,sensor.head<2>(),model);if(!det){++diagnostics.rejections["geometry"];continue;}if(!background.free(det->center)){++diagnostics.rejections["center_not_known_free"];continue;}double span=extent(group);
      if(det->strong&&(span<checks.strong_min_extent||(!checks.allow_merged_strong&&span>model.max_extent)))det->strong=false;
      if(det->strong){Points rim;std::vector<double>angles;size_t all=0;double sum=0;
        for(auto&p:group)if(p.z()<=model.rim_max_z){++all;double error=(p.head<2>()-det->center).norm()-model.radius;sum+=error*error;rim.push_back(p);if(std::abs(error)<=model.outlier){V2 d=p.head<2>()-det->center;angles.push_back(std::atan2(d.y(),d.x()));}}
        std::sort(angles.begin(),angles.end());double maxgap=2*pi;if(angles.size()>1){maxgap=angles.front()+2*pi-angles.back();for(size_t i=1;i<angles.size();++i)maxgap=std::max(maxgap,angles[i]-angles[i-1]);}
        double fraction=all?static_cast<double>(angles.size())/all:0;
        if((2*pi-maxgap)*180/pi<checks.strong_arc_min_span_deg||fraction<checks.strong_min_inlier_fraction)det->strong=false;
        if(det->strong&&checks.strong_rectangle_ratio>0&&all&&rectangle_mse(rim)<checks.strong_rectangle_ratio*sum/all)det->strong=false;
      }if(!det->strong)++diagnostics.weak_reasons["partial_or_failed_strong_checks"];detections.push_back(*det);diagnostics.strong_candidates+=det->strong;
    }
    diagnostics.candidates=detections.size();auto result=track(stamp,detections);diagnostics.tracks=tracks.size();return result;
  }
  std::shared_ptr<Track> track(double stamp,const std::vector<Detection>&detections){
    if(stamp<time){tracks.clear();selected.reset();robot_position.reset();}time=stamp;
    for(auto&t:tracks)t->predict(stamp,tracking);
    std::vector<std::tuple<double,size_t,size_t>>pairs;for(size_t i=0;i<tracks.size();++i)for(size_t j=0;j<detections.size();++j)pairs.emplace_back(tracks[i]->distance(detections[j]),i,j);std::sort(pairs.begin(),pairs.end());std::set<size_t>used_t,used_d;
    for(auto&pair:pairs){auto[distance,i,j]=pair;if(distance>tracking.gate)break;if(used_t.count(i)||used_d.count(j))continue;tracks[i]->update(stamp,detections[j],tracking);used_t.insert(i);used_d.insert(j);}
    // Weak partial-body matches cannot renew geometric identity forever.
    tracks.erase(std::remove_if(tracks.begin(),tracks.end(),[&](auto&t){return stamp-t->last_strong_update>(t->confirmed(tracking)?tracking.max_coast:tracking.tentative_coast);}),tracks.end());
    for(size_t j=0;j<detections.size();++j)if(!used_d.count(j)&&detections[j].strong&&tracks.size()<static_cast<size_t>(tracking.max_tracks))tracks.push_back(std::make_shared<Track>(stamp,detections[j],tracking));
    std::vector<std::shared_ptr<Track>>confirmed,moving;for(auto&t:tracks)if(t->confirmed(tracking)){confirmed.push_back(t);if(t->moved(tracking))moving.push_back(t);}
    bool current=std::find(confirmed.begin(),confirmed.end(),selected)!=confirmed.end();
    if(!(current&&(selected->moved(tracking)||moving.empty()))){auto candidates=moving;if(candidates.empty())for(auto&t:confirmed)if(!robot_position||(t->birth-*robot_position).norm()<=tracking.reacquire_radius)candidates.push_back(t);
      selected.reset();for(auto&t:candidates)if(!selected||std::make_pair(static_cast<double>(t->strong_hits)/t->hits,t->hits)>std::make_pair(static_cast<double>(selected->strong_hits)/selected->hits,selected->hits))selected=t;}
    if(selected&&(selected->moved(tracking)||robot_position))robot_position=selected->last_position;return selected;
  }
};
}  // namespace hsl_perception_cpp
