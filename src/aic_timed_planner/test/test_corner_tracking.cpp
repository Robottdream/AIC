#include <gtest/gtest.h>
#include <deque>
#include "aic_timed_planner/motion_model.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace aic_timed_planner;

TEST(BrakingEnvelope, CruiseRequiresBrakingAndResponseDistance) {
  ApproachLimits approach;
  const double start=6.*6./(2.*approach.deceleration)+6.*approach.response_time+approach.stop_buffer;
  EXPECT_NEAR(start,4.3,1e-9);
  EXPECT_NEAR(approach_speed(start,6.,approach),6.,1e-9);
  const double speed=approach_speed(1.2,6.,approach);
  EXPECT_LT(speed,3.);
  EXPECT_LE(speed*speed/(2.*approach.deceleration)+speed*approach.response_time,
            1.2-approach.stop_buffer+1e-9);
  EXPECT_LE(approach_speed(0.,6.,approach),.08);
}

TEST(BrakingEnvelope, DelayedChassisStopsInsideGoalInsteadOfOvershooting) {
  auto stop_position=[](bool envelope) {
    const double dt=.005;double x=0,v=6,smooth=6,command=6,delayed=6;
    std::deque<double> commands(20,6.);  // 100 ms of actuation delay.
    bool captured=false;
    for(int i=0;i<2400;++i) {
      if(i%10==0) {
        const double remaining=10.-x;
        captured=captured || remaining<=.15;
        command=captured ? 0. : (envelope ? approach_speed(remaining,6.,ApproachLimits{}) :
          std::min(6.,std::max(.08,6.*remaining/1.2)));
      }
      smooth+=std::clamp(command-smooth,-10.*dt,10.*dt);
      commands.push_back(smooth);delayed=commands.front();commands.pop_front();
      v+=(delayed-v)*dt/.05;  // Finite response of the body velocity servo.
      x+=v*dt;
    }
    EXPECT_NEAR(v,0.,1e-6);return x;
  };
  EXPECT_GT(stop_position(false),10.2);  // Reproduces the old late-braking case.
  const double improved=stop_position(true);
  EXPECT_GT(improved,9.75);EXPECT_LT(improved,10.02);
}

TEST(BrakingEnvelope, AlignmentWaitsForTranslationAndReservesAngularResponse) {
  EXPECT_DOUBLE_EQ(alignment_speed(1.,.2,4.,10.,.15),0.);
  EXPECT_DOUBLE_EQ(alignment_speed(-1.,-.2,4.,10.,.15),0.);
  const double w=alignment_speed(.1,0.,4.,10.,.15);
  EXPECT_GT(w,0.);EXPECT_LT(w,.6);
  EXPECT_NEAR(w*w/20.+w*.15,.1,1e-9);
  EXPECT_NEAR(alignment_speed(-.1,0.,4.,10.,.15),-w,1e-9);
}

TEST(BrakingEnvelope, FinalYawDoesNotReplacePositionAlignmentOutsideTolerance) {
  Pose2 goal{0,0,0};Point carrot{0,0};
  // At 18 cm, the old 20 cm phase switch selected final yaw while Nav2 still
  // required a turn toward the position (15 cm tolerance).
  EXPECT_NEAR(std::abs(alignment_heading({.18,0,0},goal,carrot,.15)),PI,1e-9);
  EXPECT_NEAR(alignment_heading({.14,0,0},goal,carrot,.15),0,1e-9);
}

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
TEST(FrameAlignment, LocalOdomPoseIsTransformedBeforeComparingWithMapPath) {
  auto clock=std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id="map";transform.child_frame_id="odom";
  transform.transform.translation.x=1.;transform.transform.translation.y=2.;
  transform.transform.rotation.z=std::sin(PI/4);transform.transform.rotation.w=std::cos(PI/4);
  ASSERT_TRUE(buffer.setTransform(transform,"test",true));
  geometry_msgs::msg::PoseStamped odom,map;
  odom.header.frame_id="odom";odom.pose.position.x=-.5;odom.pose.position.y=1.;
  odom.pose.orientation.z=std::sin(.1);odom.pose.orientation.w=std::cos(.1);
  buffer.transform(odom,map,"map",tf2::durationFromSec(.1));
  EXPECT_EQ(map.header.frame_id,"map");
  EXPECT_NEAR(map.pose.position.x,0.,1e-9);EXPECT_NEAR(map.pose.position.y,1.5,1e-9);
  const double map_yaw=.2+PI/2;
  FrameAlignment alignment{{map.pose.position.x,map.pose.position.y,map_yaw},{-.5,1.,.2}};
  auto physical_goal=alignment.toWorld({.5,1.5,map_yaw});
  EXPECT_NEAR(physical_goal.x,-.5,1e-9);EXPECT_NEAR(physical_goal.y,.5,1e-9);
  // Comparing the raw odom pose with the map goal gives the wrong distance
  // and heading, even though both incoming messages are individually valid.
  EXPECT_GT(std::hypot(.5-odom.pose.position.x,1.5-odom.pose.position.y),1.);
  EXPECT_NEAR(std::hypot(.5-map.pose.position.x,1.5-map.pose.position.y),.5,1e-9);
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
