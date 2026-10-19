#include <gtest/gtest.h>
#include "aic_timed_planner/prepared_route.hpp"
using namespace aic_timed_planner;

TEST(PreparedRoute, RejectsWrongGoalArrivalErrorMovingStartAndAge) {
  const Pose2 start{0,0,0},goal{2,1,PI};
  EXPECT_TRUE(prepared_endpoints_match(start,goal,start,goal,0.,1.));
  EXPECT_FALSE(prepared_endpoints_match({.21,0,0},goal,start,goal,0.,1.));
  EXPECT_FALSE(prepared_endpoints_match({0,0,.36},goal,start,goal,0.,1.));
  EXPECT_FALSE(prepared_endpoints_match(start,{2,1,0},start,goal,0.,1.));
  EXPECT_FALSE(prepared_endpoints_match(start,goal,start,goal,.11,1.));
  EXPECT_FALSE(prepared_endpoints_match(start,goal,start,goal,0.,30.1));
  EXPECT_FALSE(prepared_endpoints_match(start,goal,start,goal,0.,-1.));
}

TEST(PreparedRoute, ArrivalConnectorIsCheckedAndRetimesTheWholeRoute) {
  const std::vector<RouteSample> route{{0,0,0,0,0},{.2,0,0,.2,1},{.4,0,0,.4,0}};
  double delay=0;
  auto clear=[](double,double,double,double){return true;};
  EXPECT_TRUE(validate_prepared_route(route,{-.2,0,0},{0,0,0},10,10,4,10,10,clear,delay));
  EXPECT_GT(delay,.28);
  auto blocked=[](double x,double,double,double){return x<-.12 || x>-.08;};
  EXPECT_FALSE(validate_prepared_route(route,{-.2,0,0},{0,0,0},10,10,4,10,10,blocked,delay));
}

TEST(PreparedRoute, ChecksArcInteriorAndRejectsNewDynamicOccupancy) {
  const double angle=PI/8,r=.2/angle;
  const double x=r*std::sin(angle),y=r*(1.-std::cos(angle));
  const std::vector<RouteSample> route{{0,0,0,0,0},{x,y,angle,.2,1},{x,y,angle,.2,0}};
  double delay=0;int calls=0;
  auto clear=[&](double,double,double,double){++calls;return true;};
  EXPECT_TRUE(validate_prepared_route(route,{0,0,0},{0,0,0},10,10,4,10,10,clear,delay));
  EXPECT_GT(calls,10);
  // Both vertices are clear; a newly observed actor covers the arc midpoint.
  auto fresh_actor=[](double px,double,double,double t){return !(px>.07 && px<.13 && t>.05);};
  EXPECT_FALSE(validate_prepared_route(route,{0,0,0},{0,0,0},10,10,4,10,10,fresh_actor,delay));
}

TEST(PreparedRoute, WrapsHeadingAndChecksTerminalRotation) {
  EXPECT_TRUE(prepared_endpoints_match({0,0,-PI+.01},{1,0,0},{0,0,PI-.01},{1,0,0},0,1));
  const std::vector<RouteSample> route{{0,0,0,0,0},{.2,0,0,.2,0},{.2,0,PI,1.4,0}};
  double delay=0;
  auto blocked_turn=[](double,double,double yaw,double){return yaw<.8 || yaw>2.;};
  EXPECT_FALSE(validate_prepared_route(route,{0,0,0},{0,0,0},10,10,4,10,10,blocked_turn,delay));
}
