#include <gtest/gtest.h>
#include "aic_timed_planner/motion_model.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
using namespace aic_timed_planner;

TEST(FrameAlignment, WallCornerRejectsLocalizationOnlyClearance) {
  // Recorded 2026-10-14 delivery failure: the map pose clears the wall, but
  // its world footprint intersects it. Keep both checks in physical coordinates.
  nav2_costmap_2d::Costmap2D grid(640,440,.05,-16,-16,0);
  grid.setCost(279,117,254);grid.setCost(279,118,254);
  nav2_costmap_2d::Footprint fp;
  for(auto xy : {std::pair<double,double>{.34,.39},{.34,-.39},{-.34,-.39},{-.34,.39}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;fp.push_back(p);
  }
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> check(&grid);
  Pose2 map{-1.61192,-9.87953,2.4695},world{-1.652752,-9.917333,2.466183};
  FrameAlignment alignment{map,world};auto physical=alignment.toWorld(map);
  EXPECT_LT(check.footprintCostAtPose(map.x,map.y,map.yaw,fp),253);
  EXPECT_GE(check.footprintCostAtPose(physical.x,physical.y,physical.yaw,fp),253);
  // Heading change must not lose the physical position correction.
  auto turn=alignment.toWorld({map.x,map.y,1.9635});
  EXPECT_NEAR(turn.x,world.x,1e-9);EXPECT_NEAR(turn.y,world.y,1e-9);
  EXPECT_NEAR(turn.yaw,1.9635+wrap(world.yaw-map.yaw),1e-9);
}
TEST(FrameAlignment, RotationAroundRobotPreservesPathGeometry) {
  FrameAlignment alignment{{2,3,PI-.01},{-4,5,-PI+.01}};
  auto p=alignment.toWorld({3,3,PI-.01});
  EXPECT_NEAR(p.x,-4+std::cos(.02),1e-9);EXPECT_NEAR(p.y,5+std::sin(.02),1e-9);
  EXPECT_NEAR(wrap(p.yaw-(-PI+.01)),0,1e-9);
}
TEST(Kinematics, CommandNeedsAFeasibleStopBeyondItsPrefix) {
  auto stopping_x=[](double command) {
    KinematicState state{0,0,0,2.2,0};
    for(int i=0;i<40;++i) advance(state,i<6 ? command : 0,0,.05);
    EXPECT_NEAR(state.v,0,1e-9);return state.x;
  };
  // A short prefix looks clear before x=0.8, yet cruise cannot stop there.
  EXPECT_GT(stopping_x(2.2),.8);
  EXPECT_LT(stopping_x(1.1),.8);
}
TEST(Kinematics, AngularAccelerationAndBrakingRemainBounded) {
  KinematicState state{0,0,PI-.01,0,0};
  advance(state,0,4,.05);EXPECT_NEAR(state.w,.5,1e-9);
  EXPECT_NEAR(wrap(state.yaw-(PI-.01+.025)),0,1e-9);
  advance(state,0,0,.05);EXPECT_NEAR(state.w,0,1e-9);
  EXPECT_NEAR(state.x,0,1e-9);EXPECT_NEAR(state.y,0,1e-9);
}

TEST(ClearancePreference, LongerOpenRouteCanBeatWallHuggingShortcut) {
  const double shortcut_cost=1.*(1.+clearance_cost(.5,.85,3.));
  const double open_route_cost=1.2*(1.+clearance_cost(.85,.85,3.));
  EXPECT_LT(open_route_cost,shortcut_cost);
  EXPECT_DOUBLE_EQ(clearance_cost(1.2,.85,3.),0.);
  // Narrow necessary passages remain finite, and weight zero restores timing cost.
  EXPECT_TRUE(std::isfinite(clearance_cost(.34,.85,3.)));
  EXPECT_DOUBLE_EQ(clearance_cost(.34,.85,0.),0.);
}

TEST(Kinematics, EarlierStopCanBeLessSafeThanCompletingThePrefix) {
  auto stopped_at=[](int prefix_steps) {
    KinematicState state{0,0,0,2.2,0};
    for(int i=0;i<80;++i) advance(state,i<prefix_steps ? 2.2 : 0,0,.05);
    return state.x;
  };
  // An actor arrives at x=.5 after 3 s: stopping at once leaves its future lane
  // occupied; holding the command 0.3 s clears it. Test both possible stops.
  EXPECT_TRUE(overlaps(stopped_at(0),0,0,.26,.31,{.5,0},.25));
  EXPECT_FALSE(overlaps(stopped_at(6),0,0,.26,.31,{.5,0},.25));
}

TEST(Kinematics, UnsafeStationaryPoseDoesNotExcludeASafeMovingEscape) {
  KinematicState state{-3.82778,-7.39094,2.60816,0,0};Point actor{-4,-6.9};
  EXPECT_TRUE(overlaps(state.x,state.y,state.yaw,.34,.39,actor,.35));
  for(int i=0;i<80;++i) advance(state,i<20 ? 2.2 : 0,i<20 ? 1. : 0,.05);
  EXPECT_FALSE(overlaps(state.x,state.y,state.yaw,.34,.39,actor,.35));
}

TEST(DynamicPreference, ACurrentlyClearNodeCanBeSweptLater) {
  Motion actor{-2,0,2,0,.5,.35};Observation observation{0,0,1,1,true};
  auto future=forecast(actor,observation,0,5.);
  EXPECT_FALSE(overlaps(0,0,0,.34,.39,future.front(),.35));
  EXPECT_GT(future_occupancy_risk(0,0,0,.34,.39,future,0,.35),0.);
  EXPECT_DOUBLE_EQ(future_occupancy_risk(0,2,0,.34,.39,future,0,.35),0.);
}
TEST(Kinematics, BoundedRetreatLeavesAnApproachingActorsFootprint) {
  KinematicState state{-.4,-.87,-1.8,0,0};Point later{-1.2,-1.4};
  EXPECT_TRUE(overlaps(state.x,state.y,state.yaw,.34,.39,later,.35));
  for(int i=0;i<80;++i) advance(state,i<20 ? -.3 : 0,0,.05);
  EXPECT_FALSE(overlaps(state.x,state.y,state.yaw,.34,.39,later,.35));
  EXPECT_NEAR(state.v,0,1e-9);
  EXPECT_LT(std::hypot(state.x+.4,state.y+.87),.35);
}
