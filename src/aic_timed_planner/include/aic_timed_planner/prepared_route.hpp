#pragma once
#include <cmath>
#include <vector>
#include "aic_timed_planner/motion_model.hpp"

namespace aic_timed_planner
{
struct RouteSample {double x,y,yaw,t,v;};

inline bool prepared_endpoints_match(Pose2 start,Pose2 goal,Pose2 prepared_start,
                                     Pose2 prepared_goal,double speed,double age) {
  const double values[]={start.x,start.y,start.yaw,goal.x,goal.y,goal.yaw,speed,age};
  for(double value:values) if(!std::isfinite(value)) return false;
  return age>=0 && age<=30. && speed<=.10 && speed>=0 &&
    std::hypot(start.x-prepared_start.x,start.y-prepared_start.y)<=.20 &&
    std::abs(wrap(start.yaw-prepared_start.yaw))<=.35 &&
    std::hypot(goal.x-prepared_goal.x,goal.y-prepared_goal.y)<=.01 &&
    std::abs(wrap(goal.yaw-prepared_goal.yaw))<=.01;
}

// The cached search retains arc geometry and relative arrival times, not old
// dynamic forecasts. Validate vertices AND interiors with a fresh caller guard.
template<class Guard>
bool validate_prepared_route(const std::vector<RouteSample> & route,Pose2 start,
                             Pose2 prepared_start,double accel,double decel,
                             double max_w,double angular_accel,double angular_decel,
                             Guard guard,double & delay) {
  if(route.empty()) return false;
  const double d=std::hypot(start.x-prepared_start.x,start.y-prepared_start.y);
  delay=std::sqrt(2.*d*(1./accel+1./decel))+
    turn_time(prepared_start.yaw-start.yaw,max_w,angular_accel,angular_decel);
  // Short arrival-error connector, checked at <=2 cm and <=0.05 rad spacing.
  const int steps=std::max(1,int(std::ceil(std::max(d/.02,
                        std::abs(wrap(prepared_start.yaw-start.yaw))/.05))));
  for(int i=0;i<=steps;++i) {
    const double f=double(i)/steps;
    if(!guard(start.x+f*(prepared_start.x-start.x),
              start.y+f*(prepared_start.y-start.y),
              start.yaw+f*wrap(prepared_start.yaw-start.yaw),f*delay)) return false;
  }
  for(size_t i=0;i<route.size();++i) {
    const auto & n=route[i];
    if(!guard(n.x,n.y,n.yaw,n.t+delay)) return false;
    const auto before=i ? route[i-1] : RouteSample{
      prepared_start.x,prepared_start.y,prepared_start.yaw,0,0};
    const double delta=wrap(n.yaw-before.yaw);
    const double distance=std::hypot(n.x-before.x,n.y-before.y);
    // Search arcs have 0.20 m length. The final goal connector is linear,
    // exactly like the existing planner's terminal check.
    const bool arc=i>0 && i+1<route.size() && distance>.01 && std::abs(delta)>1e-6;
    const double radius=arc ? .20/delta : 0.;
    const int count=std::max(4,int(std::ceil(std::max(distance/.02,std::abs(delta)/.05))));
    for(int j=1;j<count;++j) {
      const double f=double(j)/count,yaw=before.yaw+delta*f;
      const double x=arc ? before.x+radius*(std::sin(yaw)-std::sin(before.yaw)) :
        before.x+(n.x-before.x)*f;
      const double y=arc ? before.y-radius*(std::cos(yaw)-std::cos(before.yaw)) :
        before.y+(n.y-before.y)*f;
      if(!guard(x,y,yaw,before.t+f*(n.t-before.t)+delay)) return false;
    }
  }
  return true;
}
}  // namespace aic_timed_planner
