// Offline adapter over the unchanged 41e0d7d crop/DBSCAN implementations.
#include "robot_body_filter/crop_box.hpp"
#include "dbscan_filter/dbscan.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>
extern "C" std::size_t anton_prefilter(const float * xyz, std::size_t count,
  const double * sensor_from_base, std::uint32_t * output)
{
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
  for(int r=0;r<3;++r) {
    for(int c=0;c<3;++c) {pose.linear()(r,c)=sensor_from_base[r*4+c];}
    pose.translation()[r]=sensor_from_base[r*4+3];
  }
  robot_body_filter::CropBox body,floor;
  body.min=Eigen::Vector3d(-0.25,-0.25,-1.0);body.max=Eigen::Vector3d(0.25,0.25,1.0);
  floor.min=Eigen::Vector3d(-1000,-1000,-1.0);floor.max=Eigen::Vector3d(1000,1000,0.02);
  body.applyFramePose(pose,false);floor.applyFramePose(pose,false);
  std::vector<Eigen::Vector3f> points;std::vector<std::uint32_t> indices;
  for(std::size_t i=0;i<count;++i) {
    Eigen::Vector3d p(xyz[i*3],xyz[i*3+1],xyz[i*3+2]);
    if(body.contains(p)||floor.contains(p)) {continue;}
    points.emplace_back(p.cast<float>());indices.push_back(i);
  }
  if(points.empty()||points.size()>30000) {
    for(std::size_t i=0;i<indices.size();++i) {output[i]=indices[i];}
    return indices.size();
  }
  dbscan_filter::DbscanParams params;
  params.eps=0.08;params.min_points=1;params.min_cluster_size=10;
  params.min_cluster_size_radius=1.6;params.min_range=0.05;
  std::vector<std::int32_t> labels;
  const auto clusters=dbscan_filter::cluster(points,params,labels);
  dbscan_filter::dropSmallClusters(points,labels,clusters,params);
  std::size_t kept=0;
  for(std::size_t i=0;i<labels.size();++i) {if(labels[i]>=0) {output[kept++]=indices[i];}}
  return kept;
}
