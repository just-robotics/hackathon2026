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

TEST(Detector, TallCompactObjectIsNotAStationaryRobot) {
  std::vector<Point> box;
  for (int i=0;i<=8;++i) {
    for (double z:{0.10,0.20,0.30,0.40,0.50,0.58}) {box.push_back({1.30,0.8+i*0.05,z});}
  }
  EXPECT_FALSE(detect(box,free_grid(),{0.5,1.0}));
  // The model is explicit; a taller allowed body retains this candidate.
  EXPECT_TRUE(detect(box,free_grid(),{0.5,1.0},std::nullopt,0.60));
  // A single noisy height outlier among many robot returns is tolerated.
  std::vector<Point> robot;
  for (int i=0;i<10;++i) {robot.push_back({1.322,0.96+i*.008,0.3});}
  robot.push_back({1.322,1.0,0.55});
  EXPECT_TRUE(detect(robot,free_grid(),{0.5,1.0}));
}

TEST(Detector, DiameterChecksPhysicalSpanRatherThanBoundingBoxDiagonal) {
  std::vector<Position> ring;
  std::vector<Point> hits;
  for (int i=0;i<36;++i) {
    const double angle=i*2*std::acos(-1.0)/36;
    ring.push_back({1.5+0.178*std::cos(angle),1.0+0.178*std::sin(angle)});
    hits.push_back({ring.back().x,ring.back().y,0.3});
  }
  // Bounding-box diagonal is ~0.503 m, though physical diameter is 0.356 m.
  EXPECT_TRUE(compatible_body_diameter(ring,0.476));
  EXPECT_TRUE(detect(hits,free_grid(),{0.5,1.0}));
  std::vector<Point> low_box;
  for (int i=0;i<=12;++i) {low_box.push_back({1.3,0.7+i*0.05,0.12});}
  EXPECT_FALSE(detect(low_box,free_grid(),{0.5,1.0}));
  // A small visible fragment remains ambiguous, even if stationary.
  low_box.resize(4);
  EXPECT_TRUE(detect(low_box,free_grid(),{0.5,1.0}));
}

TEST(Detector, DiameterHandlesCollinearDuplicateAndRotatedPoints) {
  EXPECT_TRUE(compatible_body_diameter({},0.476));
  EXPECT_TRUE(compatible_body_diameter({{1,1},{1,1},{1,1}},0.476));
  EXPECT_FALSE(compatible_body_diameter({{1,1},{1.3,1},{1.6,1}},0.476));
  EXPECT_FALSE(compatible_body_diameter({{1,1},{1.2122,1.2122},{1.4243,1.4243}},0.476));
}
