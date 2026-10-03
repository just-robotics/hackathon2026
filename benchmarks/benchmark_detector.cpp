// SPDX-License-Identifier: Apache-2.0
// Standalone detector timing. Input decoding and output are outside measurements.
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include "hsl_perception_cpp/detector.hpp"
using namespace hsl_perception_cpp;
template<class T> T read(std::istream & in) {T value;in.read(reinterpret_cast<char*>(&value),sizeof(value));if(!in)throw std::runtime_error("truncated input");return value;}
struct Frame {double stamp;V3 sensor;Points points;};
Detector detector(){Detector d;d.model.max_gap_share=.25;d.model.line_ratio=.50;d.checks.strong_min_extent=.25;d.checks.strong_arc_min_span_deg=75;d.checks.strong_min_inlier_fraction=.8;d.checks.allow_merged_strong=false;return d;}
int main(int argc,char**argv){try{
 if(argc!=2)throw std::runtime_error("expected prepared binary input");
 std::ifstream in(argv[1],std::ios::binary);char magic[8];in.read(magic,8);if(std::string(magic,8)!="HSLBEN01")throw std::runtime_error("bad format");
 double r=read<double>(in),ox=read<double>(in),oy=read<double>(in);auto w=read<uint32_t>(in),h=read<uint32_t>(in);std::vector<int> grid(w*h);for(auto &v:grid)v=read<int32_t>(in);
 auto count=read<uint32_t>(in);std::vector<Frame> frames;frames.reserve(count);
 for(uint32_t i=0;i<count;++i){Frame f;f.stamp=read<double>(in);for(int j=0;j<3;++j)f.sensor[j]=read<double>(in);auto n=read<uint32_t>(in);f.points.resize(n);for(auto &p:f.points)for(int j=0;j<3;++j)p[j]=read<float>(in);frames.push_back(std::move(f));}
 // Warm the full sequence, then reset tracking before the measured sequence.
 auto warm=detector();warm.background.set(r,ox,oy,w,h,grid);for(auto &f:frames)warm.step(f.points,f.sensor,f.stamp);
 auto d=detector();d.background.set(r,ox,oy,w,h,grid);
 std::cout<<std::setprecision(17)<<"[";bool first=true;
 for(auto &f:frames){auto start=std::chrono::steady_clock::now();auto t=d.step(f.points,f.sensor,f.stamp);double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();auto &a=d.diagnostics;
  if(!first)std::cout<<",";first=false;std::cout<<"{\"ms\":"<<ms<<",\"fresh\":"<<(t&&std::abs(t->last_update-f.stamp)<1e-6?"true":"false")<<",\"strong\":"<<a.strong_candidates<<",\"foreground\":"<<a.foreground_points<<",\"clusters\":"<<a.clusters<<",\"xy\":";if(t)std::cout<<"["<<t->mean[0]<<","<<t->mean[1]<<"]";else std::cout<<"null";std::cout<<"}";
 }std::cout<<"]\n";return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}}
