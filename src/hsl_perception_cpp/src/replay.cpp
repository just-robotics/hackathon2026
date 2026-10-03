// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <iomanip>
#include "hsl_perception_cpp/detector.hpp"
int main(int argc,char**argv){
  hsl_perception_cpp::Detector d;bool sim=argc>1&&std::string(argv[1])=="simulation";
  if(sim){d.model.max_gap_share=.12;d.model.line_ratio=.35;d.checks.strong_arc_min_span_deg=90;d.checks.strong_min_inlier_fraction=.95;d.checks.strong_rectangle_ratio=.7;}
  else {d.model.max_gap_share=.25;d.model.line_ratio=.50;d.checks.strong_min_extent=.25;d.checks.strong_arc_min_span_deg=75;d.checks.strong_min_inlier_fraction=.8;}
  if(argc>1&&std::string(argv[1])=="default") {d.model= hsl_perception_cpp::Model{};d.checks=hsl_perception_cpp::Checks{};}
  else {d.checks.allow_merged_strong=false;}
  for(int i=2;i<argc;i+=2){
    if(i+1>=argc)return 2;
    std::string key=argv[i];double v=std::stod(argv[i+1]);
    if(key=="--max-height")d.model.max_height=v;
    else if(key=="--max-gap-share")d.model.max_gap_share=v;
    else if(key=="--line-ratio")d.model.line_ratio=v;
    else if(key=="--arc-span")d.checks.strong_arc_min_span_deg=v;
    else if(key=="--inlier-fraction")d.checks.strong_min_inlier_fraction=v;
    else if(key=="--rectangle-ratio")d.checks.strong_rectangle_ratio=v;
    else if(key=="--min-extent")d.checks.strong_min_extent=v;
    else return 2;
  }
  double r,ox,oy;int w,h;if(!(std::cin>>r>>ox>>oy>>w>>h))return 1;
  std::vector<int>grid(w*h);for(auto&v:grid)std::cin>>v;d.background.set(r,ox,oy,w,h,grid);
  double stamp,sx,sy,sz;size_t n;
  while(std::cin>>stamp>>sx>>sy>>sz>>n){hsl_perception_cpp::Points p(n);for(auto&v:p)std::cin>>v.x()>>v.y()>>v.z();auto t=d.step(p,{sx,sy,sz},stamp);auto a=d.diagnostics;
    std::cout<<std::setprecision(16)<<"{\"foreground_points\":"<<a.foreground_points<<",\"clusters\":"<<a.clusters<<",\"strong_candidates\":"<<a.strong_candidates<<",\"tracks\":"<<a.tracks<<",\"detected\":"<<(t&&std::abs(t->last_update-stamp)<1e-6?"true":"false")<<",\"track_xy\":";
    if(t)std::cout<<"["<<t->mean[0]<<","<<t->mean[1]<<"]";else std::cout<<"null";
    std::cout<<",\"candidates\":"<<a.candidates<<",\"hits\":"<<(t?t->hits:0)<<",\"last_update\":"<<(t?t->last_update:stamp);
    auto counts=[&](const std::map<std::string,size_t>&values){std::cout<<"{";bool first=true;for(auto&[key,count]:values){if(!first)std::cout<<",";first=false;std::cout<<"\""<<key<<"\":"<<count;}std::cout<<"}";};
    std::cout<<",\"rejections\":";counts(a.rejections);std::cout<<",\"weak_reasons\":";counts(a.weak_reasons);
    std::cout<<",\"velocity\":";if(t)std::cout<<"["<<t->mean[2]<<","<<t->mean[3]<<"]";else std::cout<<"null";std::cout<<"}"<<std::endl;
  }return 0;
}
