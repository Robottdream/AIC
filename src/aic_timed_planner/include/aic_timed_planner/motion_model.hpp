#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace aic_timed_planner
{
constexpr double PI = 3.14159265358979323846;
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
struct Motion {double sx, sy, ex, ey, size, speed;};
struct Observation {double stamp=0, along=0, direction=1, gain=.965; bool valid=false;};
struct Point {double x, y;};
// A route preference only: neither speed limiting nor a new lethal boundary.
inline double clearance_cost(double clearance,double preferred,double weight) {
  const double deficit=std::max(0.,1.-clearance/std::max(.01,preferred));
  return std::max(0.,weight)*deficit*deficit;
}
struct Pose2 {double x, y, yaw;};
struct FrameAlignment {
  Pose2 map, world;
  Pose2 toWorld(Pose2 p) const {
    const double a=wrap(world.yaw-map.yaw),c=std::cos(a),s=std::sin(a);
    return {world.x+(p.x-map.x)*c-(p.y-map.y)*s,
            world.y+(p.x-map.x)*s+(p.y-map.y)*c,wrap(p.yaw+a)};
  }
};
struct KinematicState {double x,y,yaw,v,w;};
struct DriveCommand {double v,w;};
// Preserve curvature while reducing simultaneous translation/rotation, then
// limit steering against measured speed during the braking transient.
inline DriveCommand stable_command(double v,double w,double measured_v,double lateral_limit) {
  if(lateral_limit<=0) return {v,w};
  const double demand=std::abs(v*w);
  if(demand>lateral_limit) {
    const double scale=std::sqrt(lateral_limit/demand);v*=scale;w*=scale;
  }
  const double bound=lateral_limit/std::max({.01,std::abs(v),std::abs(measured_v)});
  return {v,std::clamp(w,-bound,bound)};
}
struct MotionLimits {
  double linear_accel=3.5,linear_decel=5.,angular_accel=10.,max_angular_velocity=4.,angular_decel=10.;
  bool valid() const {
    return std::isfinite(linear_accel) && linear_accel>0 &&
      std::isfinite(linear_decel) && linear_decel>0 &&
      std::isfinite(angular_accel) && angular_accel>0 &&
      std::isfinite(max_angular_velocity) && max_angular_velocity>0 &&
      std::isfinite(angular_decel) && angular_decel>0;
  }
};
// Reserve distance travelled during feedback/actuation delay before braking.
// Solves distance = speed * response_time + speed^2 / (2 * deceleration).
inline double braking_speed(double distance,double deceleration,double response_time) {
  if(distance<=0) return 0.;
  const double delay_speed=deceleration*response_time;
  return std::sqrt(delay_speed*delay_speed+2.*deceleration*distance)-delay_speed;
}
struct ApproachLimits {
  double deceleration=6.,response_time=.20,stop_buffer=.10;
  bool valid() const {
    return std::isfinite(deceleration) && deceleration>0 &&
      std::isfinite(response_time) && response_time>=0 &&
      std::isfinite(stop_buffer) && stop_buffer>=0;
  }
};
inline double approach_speed(double remaining,double cruise,const ApproachLimits & limits,
                             double minimum=.08) {
  return std::min(cruise,std::max(minimum,braking_speed(
    std::max(0.,remaining-limits.stop_buffer),limits.deceleration,limits.response_time)));
}
inline double alignment_speed(double error,double measured_linear,double maximum,
                              double angular_deceleration,double response_time) {
  // Avoid steering the still-moving chassis into an arc at the final pose.
  if(std::abs(measured_linear)>.08) return 0.;
  return std::copysign(std::min(maximum,braking_speed(
    std::abs(wrap(error)),angular_deceleration,response_time)),error);
}
inline double alignment_heading(Pose2 robot,Pose2 goal,Point carrot,double position_tolerance) {
  // Use the goal checker's position tolerance; an earlier yaw switch can leave
  // the robot stopped outside the position acceptance circle.
  return std::hypot(goal.x-robot.x,goal.y-robot.y)<position_tolerance ? goal.yaw :
    std::atan2(carrot.y-robot.y,carrot.x-robot.x);
}
inline void advance(KinematicState & state,double target_v,double target_w,double dt,const MotionLimits & limits=MotionLimits{}) {
  state.v+=std::clamp(target_v-state.v,-limits.linear_decel*dt,limits.linear_accel*dt);
  const bool reversing=state.w*target_w<0;
  if(reversing && std::abs(state.w)<limits.angular_decel*dt) {
    const double remaining=dt-std::abs(state.w)/limits.angular_decel;
    state.w=std::copysign(std::min(std::abs(target_w),limits.angular_accel*remaining),target_w);
  } else {
    const double rate=(reversing || std::abs(target_w)<std::abs(state.w)) ? limits.angular_decel : limits.angular_accel;
    state.w+=std::clamp(target_w-state.w,-rate*dt,rate*dt);
  }
  const double middle=state.yaw+state.w*dt/2.;
  state.x+=state.v*std::cos(middle)*dt;state.y+=state.v*std::sin(middle)*dt;
  state.yaw=wrap(state.yaw+state.w*dt);
}

inline std::vector<Point> forecast(const Motion & m, Observation state, double now, double horizon)
{
  const double length=std::hypot(m.ex-m.sx,m.ey-m.sy);
  const double ux=(m.ex-m.sx)/length, uy=(m.ey-m.sy)/length;
  double along=std::clamp(state.along,0.,length), direction=state.direction;
  const double advance=std::max(0.,now-state.stamp);
  std::vector<Point> result;
  double t=-advance;
  while(t<horizon+.025) {
    const double remaining=direction>0 ? length-along : along;
    if(remaining<.20) direction=-direction;
    const double distance=direction>0 ? length-along : along;
    if(t>=-.001) result.push_back({m.sx+along*ux,m.sy+along*uy});
    along+=direction*state.gain*std::min(m.speed,distance)*.025;
    along=std::clamp(along,0.,length); t+=.025;
  }
  if(result.empty()) result.push_back({m.sx+along*ux,m.sy+along*uy});
  return result;
}
inline bool overlaps(double x, double y, double yaw, double hx, double hy,
                     Point obstacle, double half)
{
  const double c=std::cos(yaw),s=std::sin(yaw),dx=obstacle.x-x,dy=obstacle.y-y;
  return std::abs(dx)<=half+hx*std::abs(c)+hy*std::abs(s) &&
    std::abs(dy)<=half+hx*std::abs(s)+hy*std::abs(c) &&
    std::abs(dx*c+dy*s)<=hx+half*(std::abs(c)+std::abs(s)) &&
    std::abs(-dx*s+dy*c)<=hy+half*(std::abs(c)+std::abs(s));
}
// Soft preference against nodes an approaching actor will soon sweep if
// tracking temporarily slows there. This is a rolling forecast, not a track mask.
inline double future_occupancy_risk(double x,double y,double yaw,double hx,double hy,
                                   const std::vector<Point> & future,double at,double half) {
  if(future.empty()) return 0.;
  double risk=0;
  for(double lead:{1.,2.,3.,4.}) {
    const auto i=std::min(future.size()-1,size_t(std::max(0.,at+lead)/.025));
    if(overlaps(x,y,yaw,hx,hy,future[i],half)) risk+=.25;
  }
  return risk;
}
inline double turn_time(double radians,double max_w=4.,double accel=10.,double decel=-1.)
{
  const double a=std::abs(wrap(radians));
  if(decel<0) decel=accel;  // Preserve the symmetric legacy overload.
  const double inverse_rates=1./accel+1./decel;
  return a<=.5*max_w*max_w*inverse_rates ? std::sqrt(2.*a*inverse_rates) : a/max_w+.5*max_w*inverse_rates;
}
}  // namespace aic_timed_planner
