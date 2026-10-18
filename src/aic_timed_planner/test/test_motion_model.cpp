#include <gtest/gtest.h>
#include "aic_timed_planner/motion_model.hpp"
using namespace aic_timed_planner;
TEST(StabilityCommand, StraightAndStationaryTurnKeepRequestedSpeed) {
  auto c=stable_command(6.,0.,6.,3.);EXPECT_DOUBLE_EQ(c.v,6.);EXPECT_DOUBLE_EQ(c.w,0.);
  c=stable_command(0.,4.,0.,3.);EXPECT_DOUBLE_EQ(c.v,0.);EXPECT_DOUBLE_EQ(c.w,4.);
}
TEST(StabilityCommand, CurvaturePreservedAtSteadyState) {
  auto c=stable_command(6.,4.,0.,3.);
  EXPECT_NEAR(std::abs(c.v*c.w),3.,1e-9);EXPECT_NEAR(c.w/c.v,4./6.,1e-9);
}
TEST(StabilityCommand, MeasuredSpeedBoundsSteeringBeforeBrakingCompletes) {
  for(double v:{-6.,6.}) for(double w:{-4.,4.}) {
    auto c=stable_command(v,w,v,3.);
    EXPECT_LE(std::abs(v*c.w),3.+1e-9);EXPECT_LE(std::abs(c.v*c.w),3.+1e-9);
    EXPECT_EQ(std::signbit(c.v),std::signbit(v));EXPECT_EQ(std::signbit(c.w),std::signbit(w));
    auto again=stable_command(c.v,c.w,v,3.);
    EXPECT_DOUBLE_EQ(again.v,c.v);EXPECT_DOUBLE_EQ(again.w,c.w);
  }
}
TEST(StabilityCommand, LegacyScenesCanLeaveCoupledLimitDisabled) {
  auto c=stable_command(6.,4.,6.,0.);EXPECT_DOUBLE_EQ(c.v,6.);EXPECT_DOUBLE_EQ(c.w,4.);
}
TEST(MotionModel, DeceleratesAndReversesAtNativeBoundary) {
  Motion m{0,0,4,0,.5,.35}; Observation o{10,3.79,1,1,true};
  const auto p=forecast(m,o,10,2);
  EXPECT_GT(p.front().x,3.78); EXPECT_LT(p.back().x,3.5);
  for(auto q:p) {EXPECT_GE(q.x,0.);EXPECT_LE(q.x,4.);EXPECT_DOUBLE_EQ(q.y,0.);}
}
TEST(MotionModel, SamePlaceCanBeClearAtAnotherArrivalTime) {
  Motion m{0,0,4,0,.5,.35}; Observation o{0,.4,1,1,true};
  auto p=forecast(m,o,0,6);
  EXPECT_TRUE(overlaps(.4,0,0,.34,.39,p.front(),.25));
  EXPECT_FALSE(overlaps(.4,0,0,.34,.39,p.back(),.25));
}
TEST(MotionModel, RotatedRectanglesAndCornerSeparation) {
  EXPECT_TRUE(overlaps(0,0,PI/4,.34,.39,{.5,0},.25));
  EXPECT_FALSE(overlaps(0,0,PI/4,.34,.39,{.9,.9},.25));
  EXPECT_TRUE(overlaps(0,0,0,.34,.39,{0,0},.25));
}
TEST(MotionModel, FastTurnTimingUsesAccelerationAndSpeed) {
  EXPECT_NEAR(turn_time(PI),PI/4+.4,1e-9);
  EXPECT_NEAR(turn_time(.4),.4,1e-9);
}

TEST(MotionModel, ConfiguredHighAccelerationAndBraking) {
  const MotionLimits limits{15.,15.,15.,8.,15.};
  KinematicState state{0,0,0,0,0};
  advance(state,8.,8.,.05,limits);
  EXPECT_NEAR(state.v,.75,1e-9);EXPECT_NEAR(state.w,.75,1e-9);
  state.v=8.;state.w=8.;advance(state,0.,0.,.05,limits);
  EXPECT_NEAR(state.v,7.25,1e-9);EXPECT_NEAR(state.w,7.25,1e-9);
}
TEST(MotionModel, ConfiguredTurnTimingTriangularAndTrapezoidal) {
  EXPECT_NEAR(turn_time(PI,8.,15.),2.*std::sqrt(PI/15.),1e-9);
  EXPECT_NEAR(turn_time(PI,2.,15.),PI/2.+2./15.,1e-9);
  EXPECT_LT(turn_time(PI,8.,15.),turn_time(PI));
}
TEST(MotionModel, RejectsInvalidPredictionLimits) {
  EXPECT_TRUE((MotionLimits{15.,15.,15.,8.,15.}.valid()));
  EXPECT_FALSE((MotionLimits{15.,0.,15.,8.,15.}.valid()));
  EXPECT_FALSE((MotionLimits{15.,15.,15.,-8.,15.}.valid()));
}

TEST(MotionModel, IndependentAngularAccelerationAndBraking) {
  const MotionLimits limits{15.,15.,40.,20.,15.};
  KinematicState state{0,0,0,0,0};advance(state,0,20.,.05,limits);
  EXPECT_NEAR(state.w,2.,1e-9);
  state.w=8.;advance(state,0,0,.05,limits);EXPECT_NEAR(state.w,7.25,1e-9);
  state.w=-8.;advance(state,0,0,.05,limits);EXPECT_NEAR(state.w,-7.25,1e-9);
  state.w=.3;advance(state,0,-20.,.05,limits);EXPECT_NEAR(state.w,-1.2,1e-9);
}
TEST(MotionModel, AsymmetricTurnTiming) {
  EXPECT_NEAR(turn_time(PI,20.,40.,15.),std::sqrt(2.*PI*(1./40.+1./15.)),1e-9);
  EXPECT_NEAR(turn_time(PI,2.,40.,15.),PI/2.+1./40.+1./15.,1e-9);
  EXPECT_GT(turn_time(PI,20.,40.,15.),turn_time(PI,20.,40.,40.));
}
