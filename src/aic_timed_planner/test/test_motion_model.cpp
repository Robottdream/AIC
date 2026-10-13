#include <gtest/gtest.h>
#include "aic_timed_planner/motion_model.hpp"
using namespace aic_timed_planner;
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
