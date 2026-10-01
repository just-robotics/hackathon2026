// Copyright 2026 Just Robotics. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace hsl_perception {
struct Point {double x, y, z;};
struct Position {double x, y;};
inline double distance(Position a, Position b) {return std::hypot(a.x-b.x, a.y-b.y);}
struct Grid {
  double resolution = 0, origin_x = 0, origin_y = 0;
  int width = 0, height = 0;
  std::vector<int8_t> data;
  std::pair<int, int> cell(double x, double y) const {
    return {int(std::floor((x-origin_x)/resolution+1e-9)),
            int(std::floor((y-origin_y)/resolution+1e-9))};
  }
  bool valid() const {return resolution > 0 && width > 0 && height > 0 &&
    data.size() == size_t(width)*size_t(height);}
  bool occupied(double x, double y) const {
    const auto [c,r] = cell(x,y);
    return c < 0 || r < 0 || c >= width || r >= height || data[r*width+c] != 0;
  }
  bool matches_static(double x, double y, double margin=0.08) const {
    const auto [col,row] = cell(x,y);
    const int radius = int(std::ceil(margin/resolution));
    for (int r=row-radius;r<=row+radius;++r) {
      for (int c=col-radius;c<=col+radius;++c) {
        if (c<0 || r<0 || c>=width || r>=height || data[r*width+c]<50) {continue;}
        if (std::hypot(x-(origin_x+(c+0.5)*resolution),
          y-(origin_y+(r+0.5)*resolution)) <= margin+resolution*0.71) {return true;}
      }
    }
    return false;
  }
  bool clear_line(Position start, Position end, double endpoint_radius=0.20) const {
    const double d=distance(start,end);
    if (d<=2*endpoint_radius) {return true;}
    const int count=int(std::ceil((d-2*endpoint_radius)/(resolution*0.5)));
    for (int i=0;i<=count;++i) {
      const double fraction=(endpoint_radius+i*(d-2*endpoint_radius)/count)/d;
      if (occupied(start.x+fraction*(end.x-start.x),start.y+fraction*(end.y-start.y))) {return false;}
    }
    return true;
  }
};
struct Detection {Position centre; size_t hits; double extent;};
inline double median(std::vector<double> values) {
  std::sort(values.begin(),values.end());
  const size_t middle=values.size()/2;
  return values.size()%2 ? values[middle] : (values[middle-1]+values[middle])/2;
}
// Initial C++ port retains the historical cluster/radial-offset estimator.
// A compact cluster is NOT a semantic robot classification. Unknown boxes
// still need a separately measured geometry/association improvement.
inline std::optional<Detection> detect(const std::vector<Point> & scan,
  const Grid & grid, Position own, std::optional<Position> previous=std::nullopt)
{
  if (!grid.valid()) {return std::nullopt;}
  constexpr double tolerance=0.23, body_radius=0.178;
  using Cell=std::pair<int,int>;
  std::vector<Point> points;
  std::map<Cell,std::vector<size_t>> bins;
  for (const auto & p:scan) {
    const double d=distance({p.x,p.y},own);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      p.z<0.08 || p.z>0.60 || d<0.25 || d>40 || grid.matches_static(p.x,p.y)) {continue;}
    bins[{int(std::floor(p.x/tolerance)),int(std::floor(p.y/tolerance))}].push_back(points.size());
    points.push_back(p);
  }
  std::vector<bool> visited(points.size(),false);
  std::vector<Detection> detections;
  for (size_t seed=0;seed<points.size();++seed) {
    if (visited[seed]) {continue;}
    visited[seed]=true;
    std::deque<size_t> queue{seed};
    std::vector<size_t> cluster;
    while (!queue.empty()) {
      const size_t index=queue.front(); queue.pop_front(); cluster.push_back(index);
      const auto p=points[index];
      const int cx=int(std::floor(p.x/tolerance)), cy=int(std::floor(p.y/tolerance));
      for (int x=cx-1;x<=cx+1;++x) {
        for (int y=cy-1;y<=cy+1;++y) {
          const auto bin=bins.find({x,y});
          if (bin==bins.end()) {continue;}
          for (const size_t candidate:bin->second) {
            if (visited[candidate]) {continue;}
            const auto q=points[candidate];
            if (std::hypot(q.x-p.x,q.y-p.y)<=tolerance) {
              visited[candidate]=true; queue.push_back(candidate);
            }
          }
        }
      }
    }
    if (cluster.size()<3) {continue;}
    double xmin=points[seed].x,xmax=xmin,ymin=points[seed].y,ymax=ymin;
    std::vector<double> centres_x,centres_y;
    for (const auto index:cluster) {
      const auto p=points[index];
      xmin=std::min(xmin,p.x); xmax=std::max(xmax,p.x);
      ymin=std::min(ymin,p.y); ymax=std::max(ymax,p.y);
      const double scale=body_radius/distance({p.x,p.y},own);
      centres_x.push_back(p.x+(p.x-own.x)*scale);
      centres_y.push_back(p.y+(p.y-own.y)*scale);
    }
    const double extent=std::hypot(xmax-xmin,ymax-ymin);
    if (extent>0.70) {continue;}
    const Position centre{median(centres_x),median(centres_y)};
    if (grid.clear_line(own,centre)) {detections.push_back({centre,cluster.size(),extent});}
  }
  if (detections.empty()) {return std::nullopt;}
  return *std::min_element(detections.begin(),detections.end(),[&](const Detection & a,const Detection & b) {
    if (previous) {return distance(a.centre,*previous)<distance(b.centre,*previous);}
    return a.hits!=b.hits ? a.hits>b.hits : distance(a.centre,own)<distance(b.centre,own);
  });
}
}  // namespace hsl_perception
