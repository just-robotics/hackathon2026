// SPDX-License-Identifier: Apache-2.0
#include <stdexcept>
#include "hsl_perception_cpp/detector.hpp"
using namespace hsl_perception_cpp;
void require(bool ok){if(!ok)throw std::runtime_error("native detector regression");}
int main(){
  // A weak fragment cannot create/confirm a robot or sustain identity forever.
  Detector d;Detection strong{{1.,1.},.02,true},weak{{1.,1.},.08,false};
  require(!d.track(0.,{strong}));require(!d.track(.1,{weak}));require(!d.track(.2,{strong}));
  auto track=d.track(.3,{strong});require(track!=nullptr);
  for(double t:{.4,.7,1.2,1.7})require(d.track(t,{weak})==track);
  require(d.track(1.81,{weak})==nullptr);require(d.tracks.empty());
  Detector stationary;for(int i=0;i<60;++i)track=stationary.track(i*.1,{strong});require(track&&track->strong_hits==60);
  // A curved body is found from its cloud; a filled tall box stays an obstacle.
  Detector shape;shape.background.set(.05,-1.,-1.,80,80,std::vector<int>(6400,0));
  Points body;
  for(double z:{.045,.08,.105})for(int i=0;i<45;++i){double a=.6*pi+.8*pi*i/44;body.emplace_back(1.+.178*std::cos(a),1.+.178*std::sin(a),z);}
  for(int i=0;i<4;++i)track=shape.step(body,{0.,1.,.31},i*.1);
  require(track&&(track->mean.head<2>()-V2{1.,1.}).norm()<.02);
  Model model;model.max_gap_share=.12;Points box;
  for(int iz=0;iz<40;++iz)for(int ix=0;ix<20;++ix)box.emplace_back(1.+.15*ix/19,1.,.01+.4*iz/39);
  require(!inspect(box,{0.,1.},model));
}
