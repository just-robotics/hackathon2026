#include <gtest/gtest.h>
#include "hsl_perception/detector.hpp"
using namespace hsl_perception;
static Grid free_grid() {return {0.1,0,0,40,30,std::vector<int8_t>(1200,0)};}
static std::vector<Point> robot_hits() {
  return {{1.322,0.95,0.3},{1.32,1.0,0.3},{1.322,1.05,0.3}};
}
TEST(Detector, StationaryRobotAndEmptyScan) {
  const auto grid=free_grid();
  EXPECT_FALSE(detect({},grid,{0.5,1.0}));
  const auto result=detect(robot_hits(),grid,{0.5,1.0});
  ASSERT_TRUE(result); EXPECT_EQ(result->hits,3u);
  EXPECT_NEAR(result->centre.x,1.5,0.015); EXPECT_NEAR(result->centre.y,1.0,0.02);
}
TEST(Detector, OccludedRobot) {
  auto grid=free_grid();
  for (int row=0;row<grid.height;++row) {grid.data[row*grid.width+10]=100;}
  EXPECT_FALSE(detect(robot_hits(),grid,{0.5,1.0}));
}
TEST(Detector, KnownWallAndSelfReturns) {
  auto grid=free_grid(); grid.data[10*40+10]=100;
  EXPECT_FALSE(detect({{1.04,1.04,0.3},{1.04,1.04,0.3},{1.04,1.04,0.3}},grid,{0.5,1.0}));
  EXPECT_FALSE(detect({{0.5,1.0,0.3},{0.5,1.0,0.3},{0.5,1.0,0.3}},grid,{0.5,1.0}));
}
TEST(Detector, NearestPriorAssociationAndUnmappedLargeObject) {
  auto hits=robot_hits();
  for (const auto p:robot_hits()) {hits.push_back({p.x+1,p.y,p.z});}
  const auto result=detect(hits,free_grid(),{0.5,1.0},Position{2.5,1.0});
  ASSERT_TRUE(result); EXPECT_NEAR(result->centre.x,2.5,0.015);
  std::vector<Point> large;
  for (int i=0;i<10;++i) {large.push_back({1.0+i*0.1,1.0,0.3});}
  EXPECT_FALSE(detect(large,free_grid(),{0.5,1.0}));
}
